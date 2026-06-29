#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <atomic>

namespace ld06 {

// ============================================================================
// Lidar Safety Node
// ============================================================================

class LidarSafetyNode : public rclcpp::Node {
 public:
  LidarSafetyNode();
  ~LidarSafetyNode() override;

 private:
  // ---- Parameters ----
  struct Config {
    std::string input_cmd_topic;
    std::string output_cmd_topic;
    std::string front_distance_topic;
    std::string safety_stop_topic;

    float cmd_timeout_sec;
    float lidar_timeout_sec;

    float stop_distance_m;
    float clear_distance_m;
    float invalid_front_timeout_sec;

    bool allow_rotation_when_blocked;
    bool allow_reverse_when_blocked;

    float max_linear_x;
    float max_angular_z;

    bool enable_slowdown;
    float slowdown_distance_m;
    float min_slowdown_factor;
  } config_;

  // ---- State ----
  geometry_msgs::msg::Twist last_cmd_;
  std::chrono::system_clock::time_point last_cmd_time_;

  float front_distance_;
  std::chrono::system_clock::time_point last_lidar_time_;
  std::chrono::system_clock::time_point last_valid_front_time_;

  bool front_blocked_;

  std::chrono::system_clock::time_point last_log_time_;

  // ---- Publishers/Subscribers ----
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr front_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // ---- Methods ----
  void declare_parameters();
  void load_parameters();

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void front_distance_callback(const std_msgs::msg::Float32::SharedPtr msg);

  void control_loop();

  geometry_msgs::msg::Twist make_stop_cmd() const;
  geometry_msgs::msg::Twist clamp_cmd(geometry_msgs::msg::Twist cmd) const;

  bool valid_front_distance() const;
  void update_blocked_state();
  float slowdown_factor() const;

  void publish_stop_state(bool active);
  void log_status(const geometry_msgs::msg::Twist& cmd, bool safety_active,
                  const std::string& reason);

  static void log_startup(rclcpp::Logger logger, const Config& cfg);
};

}  // namespace ld06
