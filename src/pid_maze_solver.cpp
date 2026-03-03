#include "distance_controller/pid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <tuple>
#include <vector>

/**
 * A waypoint-based position controller using
 * independent PID loops for x, y, and yaw.
 *
 * The controller:
 *  - Subscribes to filtered odometry
 *  - Tracks a sequence of predefined waypoints
 *  - Uses a finite state machine for motion control
 *  - Publishes velocity commands to /cmd_vel
 *
 * The robot should support holonomic motion (vx, vy, wz).
 */
class PidMazeSolver : public rclcpp::Node {
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using LaserScan = sensor_msgs::msg::LaserScan;

  struct Waypoint {
    double x;
    double y;
    double yaw;
  };

  enum class State {
    BOOTSTRAP, // wait for odom
    TRACKING,  // drive to target position
    WAITING,   // wait between track and advance
    ADVANCING, // load next waypoint
    FINISHED,  // stop after finishing all waypoint
    FAULT,     // safety stop, for future use
  };

public:
  PidMazeSolver(int scene_num)
      : Node("pid_maze_solver"), scene_num_(scene_num), wp_world_(get_wp()) {
    RCLCPP_INFO(get_logger(), "Initializing node...");

    pub_ = create_publisher<Twist>("/cmd_vel", 10);
    sub_ = create_subscription<Odometry>(
        "/odometry/filtered", 10,
        [this](Odometry::SharedPtr msg) { odom_callback(msg); });
    scan_sub_ = create_subscription<LaserScan>(
        "/scan", 10, [this](LaserScan::SharedPtr msg) { scan_callback(msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(25),
                               [this]() { motion_callback(); });

    // Set parameters for PID
    set_param();
    last_pid_time_ = now();

    RCLCPP_INFO(get_logger(), "Initialized node");
  }

  ~PidMazeSolver() { RCLCPP_INFO(get_logger(), "Terminated node"); }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    last_odom_time_ = now();

    auto &position = msg->pose.pose.position;
    auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    update_position(position.x, position.y, yaw);
    if (state_ == State::BOOTSTRAP) {
      double current_x = position.x;
      double current_y = position.y;
      double current_yaw = yaw;
      for (auto &wp : wp_world_) {
        current_x += wp.x;
        current_y += wp.y;
        current_yaw = normalize_angle(current_yaw + wp.yaw);

        wp.x = current_x;
        wp.y = current_y;
        wp.yaw = current_yaw;
      }
      start_track();
    }
  }

  void scan_callback(const LaserScan::SharedPtr msg) {
    const auto &ranges = msg->ranges;
    const float angle_min = msg->angle_min;
    const float angle_inc = msg->angle_increment;
    const size_t n = ranges.size();

    min_front_dist_ = std::numeric_limits<double>::infinity();
    min_left_dist_ = std::numeric_limits<double>::infinity();
    min_right_dist_ = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < n; ++i) {
      float r = ranges[i];
      if (std::isinf(r) || std::isnan(r)) {
        continue;
      }
      float angle = angle_min + i * angle_inc;
      // Front: roughly -10° to +10°
      if (std::abs(angle) < 0.1745) {
        min_front_dist_ = std::min(min_front_dist_, static_cast<double>(r));
      }
      // Left side: ~ +70° to +110°
      else if (angle > 0.7853 && angle < 2.356) {
        min_left_dist_ = std::min(min_left_dist_, static_cast<double>(r));
      }
      // Right side: ~ -105° to -75°
      else if (angle < -0.7853 && angle > -2.356) {
        min_right_dist_ = std::min(min_right_dist_, static_cast<double>(r));
      }
    }
  }

  void motion_callback() {
    switch (state_) {
    case State::BOOTSTRAP:
      stop_robot();
      break;
    case State::TRACKING:
      move_robot();
      break;
    case State::WAITING:
      wait_robot();
      break;
    case State::ADVANCING:
      advance_next_wp();
      break;
    case State::FINISHED:
      finish_task();
      break;
    case State::FAULT:
      recover_robot();
      break;
    }
  }

  void stop_robot() const { publish_vel(0.0, 0.0, 0.0); }

  void move_robot() {
    if (is_reached()) {
      stop_robot();
      wait_start_ = now();
      state_ = State::WAITING;
      return;
    }
    // Check odom validity
    if ((now() - last_odom_time_).seconds() > odom_timeout_) {
      RCLCPP_ERROR(get_logger(), "Odometry timeout");
      state_ = State::FAULT;
      return;
    }
    // Compute velocity using PID controllers
    auto [vx, vy, wz] = get_vel_robot();

    const double SAFE_DIST = 0.18;
    if (min_front_dist_ < SAFE_DIST) {
      // Reverse a bit
      vx -= 0.1;
      vy = 0.0;
    } else if (min_left_dist_ < SAFE_DIST) {
      // Wall on left
      wz -= 0.1;
      vy -= 0.1;
    } else if (min_right_dist_ < SAFE_DIST) {
      // Wall on right
      wz += 0.1;
      vy += 0.1;
    }

    // limit final velocity
    vx = std::clamp(vx, -0.5, 0.5);
    vy = std::clamp(vy, -0.5, 0.5);
    wz = std::clamp(wz, -1.0, 1.0);

    publish_vel(vx, vy, wz);
  }

  void wait_robot() {
    stop_robot();
    if ((now() - wait_start_).seconds() >= wait_duration_) {
      state_ = State::ADVANCING;
    }
  }

  void advance_next_wp() {
    wp_idx_++;
    reset_pid();

    if (wp_idx_ >= wp_world_.size()) {
      stop_robot();
      state_ = State::FINISHED;
      return;
    }

    start_track();
  }

  void finish_task() {
    stop_robot();
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Finished moving the all waypoints");
    rclcpp::shutdown();
  }

  void recover_robot() {
    stop_robot();

    // Check if odometry has been restored
    if ((now() - last_odom_time_).seconds() <= odom_timeout_) {
      RCLCPP_WARN(get_logger(), "Odometry restored. Recovering...");

      reset_pid();
      state_ = State::TRACKING;
    }
  }

  void start_track() {
    auto &wp = wp_world_[wp_idx_];
    update_target(wp.x, wp.y, wp.yaw);
    state_ = State::TRACKING;
    RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                wp_idx_, target_x_, target_y_, target_yaw_);
  }

  void reset_pid() {
    pid_x_.reset();
    pid_y_.reset();
    pid_yaw_.reset();
    last_pid_time_ = now();
  }

  bool is_reached() const {
    auto [err_x, err_y, err_yaw] = get_err();
    return std::hypot(err_x, err_y) < 0.05 && std::abs(err_yaw) < 0.08;
  }

  std::tuple<double, double, double> get_err() const {
    double err_x = target_x_ - x_;
    double err_y = target_y_ - y_;
    double err_yaw = normalize_angle(target_yaw_ - yaw_);

    return {err_x, err_y, err_yaw};
  }

  double normalize_angle(double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void update_position(double x, double y, double yaw) {
    x_ = x;
    y_ = y;
    yaw_ = normalize_angle(yaw);
  }

  void update_target(double x, double y, double yaw) {
    target_x_ = x;
    target_y_ = y;
    target_yaw_ = normalize_angle(yaw);
  }

  /**
   * - Compute velocity command {vx, vy, wz} in robot frame:
   * - Compute position and yaw error in world frame
   * - Transform position error into robot frame
   * - Compute PID outputs for x, y, and yaw
   */
  std::tuple<double, double, double> get_vel_robot() {
    auto [dx, dy, dphi] = get_err();
    // Transform world to robot frame, using current robot yaw
    double err_x = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    double err_y = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

    rclcpp::Time now_t = now();
    double dt = (now_t - last_pid_time_).seconds();
    last_pid_time_ = now_t;

    // Compute pid for each x, y, and yaw with feed-forward
    double vx = pid_x_.compute(err_x, dt, 0.1);
    double vy = pid_y_.compute(err_y, dt, 0.1);
    double wz = pid_yaw_.compute(dphi, dt, 0.1);

    return {vx, vy, wz};
  }

  void publish_vel(double vx, double vy, double wz) const {
    Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    pub_->publish(cmd);
  }

  std::vector<Waypoint> get_wp() const {
    // Waypoints are defined in relative world frame (x, y, yaw)
    switch (scene_num_) {
    case 1: // For simulation
      return get_sim_wp();
    case 2: // For real cyber world
      return get_real_wp();
    default:
      RCLCPP_ERROR(get_logger(), "Undefined scene number %d", scene_num_);
      return {{0.0, 0.0, 0.0}};
    }
  }

  std::vector<Waypoint> get_sim_wp() const {
    return {{0.35, 0.0, 0.0},    {0.0, 0.0, -0.7853}, {0.2, -0.2, 0.0},
            {0.0, 0.0, -0.7853}, {0.0, -1.2, 0.0},    {0.0, 0.0, 1.5707},
            {0.5, 0.0, 0.0},     {0.0, 0.0, 1.5707},  {0.0, 0.6, 0.0},
            {0.45, 0.0, 0.0},    {0.0, 0.5, 0.0},     {0.55, 0.0, 0.0},
            {0.0, 0.9, 0.0},     {0.0, 0.0, 1.5707},  {-0.5, 0.0, 0.0},
            {0.0, -0.35, 0.0},   {-0.5, 0.0, 0.0},    {0.0, 0.0, -0.7853},
            {-0.3, 0.3, 0.0},    {0.0, 0.0, 0.7853},  {-0.6, 0.0, 0.0},
            {0.0, 0.0, 3.1415}};
  }

  std::vector<Waypoint> get_real_wp() const {
    return {
        {0.9, 0.0, 0.0}, {0.0, -0.6, 0.0}, {0.0, 0.6, 0.0}, {-0.9, 0.0, 0.0}};
  }

  void set_param() {
    pid_x_.set_gain(declare_parameter<double>("pid_x.kp", 1.1),
                    declare_parameter<double>("pid_x.ki", 0.0),
                    declare_parameter<double>("pid_x.kd", 0.05));

    pid_y_.set_gain(declare_parameter<double>("pid_y.kp", 1.1),
                    declare_parameter<double>("pid_y.ki", 0.0),
                    declare_parameter<double>("pid_y.kd", 0.05));

    pid_yaw_.set_gain(declare_parameter<double>("pid_yaw.kp", 1.5),
                      declare_parameter<double>("pid_yaw.ki", 0.0),
                      declare_parameter<double>("pid_yaw.kd", 0.1));

    double max_v = declare_parameter<double>("max_v", 0.5);
    double max_w = declare_parameter<double>("max_w", 1.0);
    pid_x_.set_limit(max_v, 5.0);
    pid_y_.set_limit(max_v, 5.0);
    pid_yaw_.set_limit(max_w, 5.0);
  }

  int scene_num_; // scene 1 for simulation, while 2 for cyber world

  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double min_front_dist_ = std::numeric_limits<double>::infinity();
  double min_left_dist_ = std::numeric_limits<double>::infinity();
  double min_right_dist_ = std::numeric_limits<double>::infinity();

  rclcpp::Time last_odom_time_;
  double odom_timeout_{0.5}; // in second

  State state_{State::BOOTSTRAP};
  rclcpp::Time wait_start_;
  double wait_duration_{2.0}; // in second

  distance_controller::PID pid_x_{};
  distance_controller::PID pid_y_{};
  distance_controller::PID pid_yaw_{};
  rclcpp::Time last_pid_time_;

  std::vector<Waypoint> wp_world_{};
  size_t wp_idx_{0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  int scene_num = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_num = std::atoi(argv[1]);
  }
  rclcpp::spin(std::make_shared<PidMazeSolver>(scene_num));
  rclcpp::shutdown();
  return 0;
}