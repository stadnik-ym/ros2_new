#include "../include/buzzer/buzzer_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

namespace buzzer_node
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
BuzzerNode::BuzzerNode(const rclcpp::NodeOptions & options)
: Node("buzzer_node", options)
{
  this->declare_parameter<std::string>("gpio_chip",       "gpiochip0");
  this->declare_parameter<int>        ("gpio_line",       17);
  this->declare_parameter<int>        ("beep_duration_ms",  500);
  this->declare_parameter<int>        ("beep_frequency_hz", 2000);

  std::string chip_name = this->get_parameter("gpio_chip").as_string();
  const auto  line_no   = static_cast<unsigned int>(
                            this->get_parameter("gpio_line").as_int());
  beep_duration_ms_  = this->get_parameter("beep_duration_ms").as_int();
  beep_frequency_hz_ = this->get_parameter("beep_frequency_hz").as_int();

  // v2 API requires the full /dev/ path
  if (chip_name.rfind("/dev/", 0) != 0) {
    chip_name = "/dev/" + chip_name;
  }

  init_gpio(chip_name, line_no);

  sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "buzzer", rclcpp::QoS(10),
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      buzzer_callback(msg);
    });

  RCLCPP_INFO(this->get_logger(),
    "BuzzerNode ready  chip=%s  line=%u  beep=%d ms @ %d Hz",
    chip_name.c_str(), line_no, beep_duration_ms_, beep_frequency_hz_);
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
BuzzerNode::~BuzzerNode()
{
  stop_beep();
  if (beep_thread_.joinable()) {
    beep_thread_.join();
  }
  // gpiod::line_request releases the line(s) automatically on destruction
  RCLCPP_INFO(this->get_logger(), "BuzzerNode shutdown, GPIO released.");
}

// ---------------------------------------------------------------------------
// GPIO init — libgpiod 2.x C++ API (request_builder pattern)
// ---------------------------------------------------------------------------
void BuzzerNode::init_gpio(const std::string & chip_path, unsigned int line_offset)
{
  line_offset_ = line_offset;

  try {
    gpiod::chip chip(chip_path);

    gpiod::line_settings settings;
    settings.set_direction(gpiod::line::direction::OUTPUT);
    settings.set_output_value(gpiod::line::value::INACTIVE);

    request_.emplace(
      chip.prepare_request()
        .set_consumer("buzzer_node")
        .add_line_settings(line_offset_, settings)
        .do_request());

  } catch (const std::exception & e) {
    throw std::runtime_error(
      "Failed to request GPIO line " + std::to_string(line_offset) +
      " on " + chip_path + ": " + e.what());
  }
}

// ---------------------------------------------------------------------------
// Callback
// ---------------------------------------------------------------------------
void BuzzerNode::buzzer_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_DEBUG(this->get_logger(), "Buzz ON");
    start_beep();
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Buzz OFF");
    stop_beep();
  }
}

// ---------------------------------------------------------------------------
// Beep control
// ---------------------------------------------------------------------------
void BuzzerNode::start_beep()
{
  bool expected = false;
  if (!buzzing_.compare_exchange_strong(expected, true)) {
    return;  // already running
  }

  if (beep_thread_.joinable()) {
    beep_thread_.join();
  }

  beep_thread_ = std::thread([this] { beep_loop(); });
}

void BuzzerNode::stop_beep()
{
  buzzing_.store(false);
  set_gpio(false);  // drive LOW immediately
}

void BuzzerNode::beep_loop()
{
  // Software PWM — toggle at beep_frequency_hz_
  const int half_period_us = 1'000'000 / (2 * beep_frequency_hz_);

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(beep_duration_ms_);

  bool level = false;
  while (buzzing_.load() && std::chrono::steady_clock::now() < deadline) {
    level = !level;
    set_gpio(level);
    std::this_thread::sleep_for(std::chrono::microseconds(half_period_us));
  }

  set_gpio(false);
  buzzing_.store(false);
}

// ---------------------------------------------------------------------------
// GPIO write — gpiod::line_request::set_value(offset, value)
// ---------------------------------------------------------------------------
void BuzzerNode::set_gpio(bool high)
{
  if (!request_) {
    return;
  }
  request_->set_value(
    line_offset_,
    high ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE);
}

}  // namespace buzzer_node

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<buzzer_node::BuzzerNode>());
  rclcpp::shutdown();
  return 0;
}
