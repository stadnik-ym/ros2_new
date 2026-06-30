#include "../include/ld06_lidar/ld06_safety_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ld06 {

LidarSafetyNode::LidarSafetyNode() : rclcpp::Node("lidar_safety") {
  declare_parameters();
  load_parameters();

  // Initialize state
  last_cmd_time_ = std::chrono::system_clock::now();
  last_lidar_time_ = std::chrono::system_clock::now();
  last_valid_front_time_ = std::chrono::system_clock::now() -
                            std::chrono::seconds(10);  // old timestamp
  front_distance_ = -1.0;
  front_blocked_ = false;
  last_log_time_ = std::chrono::system_clock::now();

  // Buzzer: start with an "old" timestamp so the first beep fires
  // immediately once a non-NONE mode is reached, instead of waiting
  // a full interval after startup.
  last_buzzer_pub_time_ = std::chrono::system_clock::now() -
                           std::chrono::seconds(10);
  last_buzzer_mode_ = BuzzerMode::NONE;

  // Create subscriptions
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      config_.input_cmd_topic, 1,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->cmd_callback(msg);
      });

  front_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      config_.front_distance_topic, 1,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        this->front_distance_callback(msg);
      });

  // Create publishers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      config_.output_cmd_topic, 1);
  stop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      config_.safety_stop_topic, 10);
  buzzer_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      config_.buzzer_topic, 10);

  // Create control timer (100 Hz)
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { this->control_loop(); });

  log_startup(this->get_logger(), config_);
}

LidarSafetyNode::~LidarSafetyNode() {}

void LidarSafetyNode::declare_parameters() {
  this->declare_parameter("input_cmd_topic", "/cmd_vel_raw");
  this->declare_parameter("output_cmd_topic", "/cmd_vel");
  this->declare_parameter("front_distance_topic", "/lidar/front_distance");
  this->declare_parameter("safety_stop_topic", "/safety/lidar_stop");

  this->declare_parameter("cmd_timeout_sec", 0.5);
  this->declare_parameter("lidar_timeout_sec", 0.7);

  this->declare_parameter("stop_distance_m", 0.22);
  this->declare_parameter("clear_distance_m", 0.30);
  this->declare_parameter("invalid_front_timeout_sec", 1.0);

  this->declare_parameter("allow_rotation_when_blocked", true);
  this->declare_parameter("allow_reverse_when_blocked", true);

  this->declare_parameter("max_linear_x", 0.35);
  this->declare_parameter("max_angular_z", 1.5);

  this->declare_parameter("enable_slowdown", true);
  this->declare_parameter("slowdown_distance_m", 0.80);
  this->declare_parameter("min_slowdown_factor", 0.25);

  // Buzzer
  this->declare_parameter("buzzer_topic", "/buzzer");
  this->declare_parameter("buzzer_interval_blocked_sec", 2.0);
  this->declare_parameter("buzzer_interval_critical_sec", 0.5);
  this->declare_parameter("buzzer_interval_slowdown_sec", 4.0);
}

void LidarSafetyNode::load_parameters() {
  config_.input_cmd_topic =
      this->get_parameter("input_cmd_topic").as_string();
  config_.output_cmd_topic =
      this->get_parameter("output_cmd_topic").as_string();
  config_.front_distance_topic =
      this->get_parameter("front_distance_topic").as_string();
  config_.safety_stop_topic =
      this->get_parameter("safety_stop_topic").as_string();

  config_.cmd_timeout_sec =
      static_cast<float>(this->get_parameter("cmd_timeout_sec").as_double());
  config_.lidar_timeout_sec =
      static_cast<float>(this->get_parameter("lidar_timeout_sec").as_double());

  config_.stop_distance_m =
      static_cast<float>(this->get_parameter("stop_distance_m").as_double());
  config_.clear_distance_m =
      static_cast<float>(this->get_parameter("clear_distance_m").as_double());
  config_.invalid_front_timeout_sec = static_cast<float>(
      this->get_parameter("invalid_front_timeout_sec").as_double());

  config_.allow_rotation_when_blocked =
      this->get_parameter("allow_rotation_when_blocked").as_bool();
  config_.allow_reverse_when_blocked =
      this->get_parameter("allow_reverse_when_blocked").as_bool();

  config_.max_linear_x = std::abs(
      static_cast<float>(this->get_parameter("max_linear_x").as_double()));
  config_.max_angular_z = std::abs(
      static_cast<float>(this->get_parameter("max_angular_z").as_double()));

  config_.enable_slowdown =
      this->get_parameter("enable_slowdown").as_bool();
  config_.slowdown_distance_m =
      static_cast<float>(this->get_parameter("slowdown_distance_m").as_double());
  config_.min_slowdown_factor =
      static_cast<float>(this->get_parameter("min_slowdown_factor").as_double());

  // Clamp min_slowdown_factor to [0, 1]
  config_.min_slowdown_factor =
      std::max(0.0f, std::min(1.0f, config_.min_slowdown_factor));

  // Buzzer
  config_.buzzer_topic = this->get_parameter("buzzer_topic").as_string();
  config_.buzzer_interval_blocked_sec = static_cast<float>(
      this->get_parameter("buzzer_interval_blocked_sec").as_double());
  config_.buzzer_interval_critical_sec = static_cast<float>(
      this->get_parameter("buzzer_interval_critical_sec").as_double());
  config_.buzzer_interval_slowdown_sec = static_cast<float>(
      this->get_parameter("buzzer_interval_slowdown_sec").as_double());
}

void LidarSafetyNode::cmd_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_ = *msg;
  last_cmd_time_ = std::chrono::system_clock::now();
}

void LidarSafetyNode::front_distance_callback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  auto now = std::chrono::system_clock::now();
  last_lidar_time_ = now;

  float distance = msg->data;

  if (distance >= 0.0f) {
    front_distance_ = distance;
    last_valid_front_time_ = now;

    if (distance <= config_.stop_distance_m) {
      front_blocked_ = true;

      if (last_cmd_.linear.x > 0.0) {
        auto stop_cmd = make_stop_cmd();

        if (config_.allow_rotation_when_blocked) {
          stop_cmd.angular.z = last_cmd_.angular.z;
        }

        cmd_pub_->publish(stop_cmd);
        publish_stop_state(true);
      }
    }
  }
}

geometry_msgs::msg::Twist LidarSafetyNode::make_stop_cmd() const {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  return msg;
}

geometry_msgs::msg::Twist LidarSafetyNode::clamp_cmd(
    geometry_msgs::msg::Twist cmd) const {
  if (cmd.linear.x > config_.max_linear_x) {
    cmd.linear.x = config_.max_linear_x;
  } else if (cmd.linear.x < -config_.max_linear_x) {
    cmd.linear.x = -config_.max_linear_x;
  }

  if (cmd.angular.z > config_.max_angular_z) {
    cmd.angular.z = config_.max_angular_z;
  } else if (cmd.angular.z < -config_.max_angular_z) {
    cmd.angular.z = -config_.max_angular_z;
  }

  // Differential robot doesn't move sideways
  cmd.linear.y = 0.0;

  return cmd;
}

bool LidarSafetyNode::valid_front_distance() const {
  auto now = std::chrono::system_clock::now();
  auto age = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - last_valid_front_time_);
  return front_distance_ >= 0.0f &&
         age.count() <= config_.invalid_front_timeout_sec;
}

void LidarSafetyNode::update_blocked_state() {
  // Hysteresis logic
  if (!valid_front_distance()) {
    // If data temporarily missing, keep blocked state
    return;
  }

  if (front_blocked_) {
    if (front_distance_ >= config_.clear_distance_m) {
      front_blocked_ = false;
    }
  } else {
    if (front_distance_ <= config_.stop_distance_m) {
      front_blocked_ = true;
    }
  }
}

float LidarSafetyNode::slowdown_factor() const {
  if (!config_.enable_slowdown) {
    return 1.0f;
  }

  if (!valid_front_distance()) {
    return 1.0f;
  }

  if (front_distance_ >= config_.slowdown_distance_m) {
    return 1.0f;
  }

  if (config_.slowdown_distance_m <= 0.01f) {
    return 1.0f;
  }

  float factor = front_distance_ / config_.slowdown_distance_m;
  factor = std::max(config_.min_slowdown_factor,
                    std::min(1.0f, factor));
  return factor;
}

void LidarSafetyNode::publish_stop_state(bool active) {
  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = active;
  stop_pub_->publish(std::move(msg));
}

// ---------------------------------------------------------------------------
// Buzzer
// ---------------------------------------------------------------------------
// Publishes a single `true` pulse on the buzzer topic whenever the
// configured interval for the current mode has elapsed. The buzzer_node
// itself is responsible for turning the pin off after its own beep
// duration, so this only needs to fire `true` periodically — it never
// needs to publish `false` to silence it between beeps.
//
// If `mode` is NONE, nothing is published and the "last beep" timer is
// reset so that if a louder mode kicks in later, it beeps right away
// rather than waiting out a stale interval from a previous mode.
void LidarSafetyNode::update_buzzer(BuzzerMode mode) {
  if (mode == BuzzerMode::NONE) {
    last_buzzer_mode_ = BuzzerMode::NONE;
    return;
  }

  float interval_sec = 0.0f;
  switch (mode) {
    case BuzzerMode::CRITICAL:
      interval_sec = config_.buzzer_interval_critical_sec;
      break;
    case BuzzerMode::BLOCKED:
      interval_sec = config_.buzzer_interval_blocked_sec;
      break;
    case BuzzerMode::SLOWDOWN:
      interval_sec = config_.buzzer_interval_slowdown_sec;
      break;
    case BuzzerMode::NONE:
      return;  // unreachable, handled above
  }

  auto now = std::chrono::system_clock::now();

  // If the mode just changed (e.g. SLOWDOWN -> BLOCKED -> CRITICAL),
  // beep immediately instead of waiting for the new interval to elapse,
  // since a mode escalation is itself worth an immediate alert.
  bool mode_changed = (mode != last_buzzer_mode_);

  auto since_last = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - last_buzzer_pub_time_);

  if (mode_changed || since_last.count() >= interval_sec) {
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = true;
    buzzer_pub_->publish(std::move(msg));
    last_buzzer_pub_time_ = now;
  }

  last_buzzer_mode_ = mode;
}

void LidarSafetyNode::control_loop() {
  auto now = std::chrono::system_clock::now();

  // Check cmd timeout
  auto cmd_age = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - last_cmd_time_);
  if (cmd_age.count() > config_.cmd_timeout_sec) {
    cmd_pub_->publish(make_stop_cmd());
    publish_stop_state(true);
    update_buzzer(BuzzerMode::CRITICAL);
    RCLCPP_WARN(this->get_logger(), "CMD TIMEOUT: stopping robot");
    return;
  }

  // Check lidar timeout
  auto lidar_age = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - last_lidar_time_);
  if (lidar_age.count() > config_.lidar_timeout_sec) {
    cmd_pub_->publish(make_stop_cmd());
    publish_stop_state(true);
    update_buzzer(BuzzerMode::CRITICAL);
    RCLCPP_WARN(this->get_logger(), "LIDAR TIMEOUT: stopping robot");
    return;
  }

  // Update safety state
  update_blocked_state();

  // Copy command
  auto cmd = last_cmd_;
  cmd = clamp_cmd(cmd);

  bool moving_forward = cmd.linear.x > 0.0f;
  bool moving_backward = cmd.linear.x < 0.0f;

  bool safety_active = false;
  std::string reason = "OK";
  BuzzerMode buzzer_mode = BuzzerMode::NONE;

  // Safety logic
  if (front_blocked_ && moving_forward) {
    cmd.linear.x = 0.0f;
    safety_active = true;
    reason = "front blocked";
    buzzer_mode = BuzzerMode::BLOCKED;

    if (!config_.allow_rotation_when_blocked) {
      cmd.angular.z = 0.0f;
    }
  }

  if (front_blocked_ && moving_backward &&
      !config_.allow_reverse_when_blocked) {
    cmd.linear.x = 0.0f;
    safety_active = true;
    reason = "reverse disabled while blocked";
    buzzer_mode = BuzzerMode::BLOCKED;
  }

  if (!front_blocked_ && moving_forward) {
    float factor = slowdown_factor();
    if (factor < 1.0f) {
      cmd.linear.x *= factor;
      safety_active = true;
      std::ostringstream ss;
      ss << "slowdown " << std::fixed << std::setprecision(2) << factor;
      reason = ss.str();
      buzzer_mode = BuzzerMode::SLOWDOWN;
    }
  }

  cmd = clamp_cmd(cmd);

  cmd_pub_->publish(cmd);
  publish_stop_state(safety_active);
  update_buzzer(buzzer_mode);

  log_status(cmd, safety_active, reason);
}

void LidarSafetyNode::log_status(const geometry_msgs::msg::Twist& cmd,
                                  bool safety_active,
                                  const std::string& reason) {
  auto now = std::chrono::system_clock::now();
  auto log_age = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - last_log_time_);

  if (log_age.count() < 0.5f) {
    return;
  }

  last_log_time_ = now;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);

  if (safety_active) {
    ss << "SAFETY: " << reason << " | front=" << front_distance_ << " m | "
       << "out linear.x=" << cmd.linear.x << ", angular.z="
       << cmd.angular.z;
    RCLCPP_WARN(this->get_logger(), "%s", ss.str().c_str());
  } else {
    ss << "OK | front=" << front_distance_ << " m | "
       << "out linear.x=" << cmd.linear.x << ", angular.z="
       << cmd.angular.z;
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
}

void LidarSafetyNode::log_startup(rclcpp::Logger logger, const Config& cfg) {
  RCLCPP_INFO(logger,
              "Lidar safety started: %s -> %s, front_topic=%s, "
              "stop=%.2f, clear=%.2f, buzzer_topic=%s",
              cfg.input_cmd_topic.c_str(), cfg.output_cmd_topic.c_str(),
              cfg.front_distance_topic.c_str(), cfg.stop_distance_m,
              cfg.clear_distance_m, cfg.buzzer_topic.c_str());
}

}  // namespace ld06

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ld06::LidarSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
