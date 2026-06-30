#pragma once

#include <atomic>
#include <optional>
#include <string>
#include <thread>

#include <gpiod.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace buzzer_node
{

class BuzzerNode : public rclcpp::Node
{
public:
  explicit BuzzerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BuzzerNode() override;

private:
  void init_gpio(const std::string & chip_path, unsigned int line_offset);
  void set_gpio(bool high);

  void buzzer_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_beep();
  void stop_beep();
  void beep_loop();

  // libgpiod v2 C++ API. line_request has no default constructor but is
  // move-constructible, so std::optional is used to defer construction
  // until init_gpio() runs (and to keep the type complete here so its
  // destructor can be generated for std::optional's internals).
  std::optional<gpiod::line_request> request_;
  unsigned int line_offset_{0};

  // ROS
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  // Config
  int beep_duration_ms_;
  int beep_frequency_hz_;

  // Beep thread
  std::thread       beep_thread_;
  std::atomic<bool> buzzing_{false};
};

}  // namespace buzzer_node
