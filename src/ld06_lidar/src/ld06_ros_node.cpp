#include "../include/ld06_lidar/ld06_ros_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <memory>

namespace ld06 {

// ============================================================================
// LD06Node Implementation
// ============================================================================

LD06Node::LD06Node() : rclcpp::Node("ld06_node") {
  declare_parameters();
  load_parameters();

  // Validate serial port can be opened
  try {
    serial_reader_ = std::make_unique<SerialReader>(config_.port,
                                                      config_.baudrate);
    if (!serial_reader_->is_open()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open serial port: %s", config_.port.c_str());
      throw std::runtime_error("Serial port open failed");
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Serial initialization failed: %s",
                 e.what());
    throw;
  }

  init_processing_components();
  init_publishers();
  start_read_thread();

  // Setup publish timer
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(config_.publish_rate_hz,
                                                     1.0f)));
  publish_timer_ = this->create_wall_timer(
      period, [this]() { this->publish_data(); });

  log_startup(this->get_logger(), config_);
}

LD06Node::~LD06Node() {
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void LD06Node::declare_parameters() {
  this->declare_parameter("port", "/dev/ttyUSB0");
  this->declare_parameter("baudrate", 230400);
  this->declare_parameter("frame_id", "laser_frame");

  this->declare_parameter("scan_topic", "/scan");
  this->declare_parameter("front_distance_topic", "/lidar/front_distance");
  this->declare_parameter("closest_topic", "/lidar/closest");

  this->declare_parameter("publish_rate_hz", 50.0);
  this->declare_parameter("range_min_m", 0.05);
  this->declare_parameter("range_max_m", 12.0);
  this->declare_parameter("min_confidence", 0);

  this->declare_parameter("angle_resolution_deg", 1.0);
  this->declare_parameter("angle_offset_deg", 0.0);
  this->declare_parameter("invert_angle_direction", false);

  this->declare_parameter("front_min_deg", -35.0);
  this->declare_parameter("front_max_deg", 35.0);
  this->declare_parameter("point_max_age_sec", 0.35);
  this->declare_parameter("front_min_points", 1);

  this->declare_parameter("front_hold_sec", 0.8);
  this->declare_parameter("front_filter_window", 5);

  this->declare_parameter("immediate_front_publish", true);
  this->declare_parameter("immediate_front_pub_min_period", 0.005);
}

void LD06Node::load_parameters() {
  config_.port = this->get_parameter("port").as_string();
  config_.baudrate = this->get_parameter("baudrate").as_int();
  config_.frame_id = this->get_parameter("frame_id").as_string();

  config_.scan_topic = this->get_parameter("scan_topic").as_string();
  config_.front_distance_topic =
      this->get_parameter("front_distance_topic").as_string();
  config_.closest_topic = this->get_parameter("closest_topic").as_string();

  config_.publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
  config_.range_min_m = this->get_parameter("range_min_m").as_double();
  config_.range_max_m = this->get_parameter("range_max_m").as_double();
  config_.min_confidence = this->get_parameter("min_confidence").as_int();

  config_.angle_resolution_deg =
      this->get_parameter("angle_resolution_deg").as_double();
  config_.angle_offset_deg =
      this->get_parameter("angle_offset_deg").as_double();
  config_.invert_angle_direction =
      this->get_parameter("invert_angle_direction").as_bool();

  config_.front_min_deg = this->get_parameter("front_min_deg").as_double();
  config_.front_max_deg = this->get_parameter("front_max_deg").as_double();
  config_.point_max_age_sec =
      this->get_parameter("point_max_age_sec").as_double();
  config_.front_min_points =
      this->get_parameter("front_min_points").as_int();

  config_.front_hold_sec = this->get_parameter("front_hold_sec").as_double();
  config_.front_filter_window =
      this->get_parameter("front_filter_window").as_int();

  config_.immediate_front_publish =
      this->get_parameter("immediate_front_publish").as_bool();
  config_.immediate_front_pub_min_period =
      this->get_parameter("immediate_front_pub_min_period").as_double();
}

void LD06Node::init_publishers() {
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      config_.scan_topic, 10);
  front_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      config_.front_distance_topic, 10);
  closest_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      config_.closest_topic, 10);
}

void LD06Node::init_processing_components() {
  crc_calc_ = std::make_unique<CRC8Calculator>();
  parser_ = std::make_unique<PacketParser>(*crc_calc_);

  int bin_count = static_cast<int>(std::round(360.0 /
                                               config_.angle_resolution_deg));
  scan_buffer_ = std::make_unique<ScanBuffer>(bin_count);
  front_analyzer_ = std::make_unique<FrontSectorAnalyzer>(
      config_.front_min_deg, config_.front_max_deg,
      config_.front_filter_window, config_.front_hold_sec,
      config_.front_min_points);
  closest_finder_ = std::make_unique<ClosestPointFinder>();
}

void LD06Node::start_read_thread() {
  read_thread_ = std::thread([this]() { this->read_loop(); });
}

void LD06Node::read_loop() {
  std::vector<uint8_t> packet;

  while (running_ && rclcpp::ok()) {
    try {
      if (!serial_reader_->read_packet(packet)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }

      auto packet_data = parser_->parse(packet.data(), packet.size());
      if (!packet_data) {
        continue;
      }

      auto now = std::chrono::system_clock::now();
      float front_packet_min = INFINITY;
      int front_packet_points = 0;

      // Process points from packet
      for (const auto& point : packet_data->points) {
        // Filter by confidence
        if (point.confidence < config_.min_confidence) {
          continue;
        }

        // Filter by range
        if (point.distance_m < config_.range_min_m ||
            point.distance_m > config_.range_max_m) {
          continue;
        }

        // Apply angle correction
        float robot_angle = corrected_angle(point.raw_angle_deg);
        int idx = angle_to_index(robot_angle);

        scan_buffer_->update_bin(idx, point.distance_m, point.confidence,
                                  now);

        // Track front sector
        float angle_180 = angle_utils::normalize_180(robot_angle);
        if (front_analyzer_->angle_in_front(angle_180)) {
          front_packet_points++;
          if (point.distance_m < front_packet_min) {
            front_packet_min = point.distance_m;
          }
        }
      }

      // Immediate front publish
      if (config_.immediate_front_publish &&
          front_packet_points >= config_.front_min_points &&
          !std::isinf(front_packet_min)) {
        publish_front_immediate(front_packet_min);
      }

    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Read loop error: %s", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

void LD06Node::publish_data() {
  auto now = std::chrono::system_clock::now();

  // Get scan snapshot
  auto snapshot = scan_buffer_->get_snapshot(config_.point_max_age_sec, now);

  // Build LaserScan message
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header.stamp = this->now();
  scan_msg->header.frame_id = config_.frame_id;

  scan_msg->angle_min = 0.0f;
  scan_msg->angle_max = 2.0f * M_PI;
  scan_msg->angle_increment =
      static_cast<float>(config_.angle_resolution_deg * M_PI / 180.0);

  scan_msg->time_increment = 0.0f;
  scan_msg->scan_time = 1.0f / std::max(config_.publish_rate_hz, 1.0f);

  scan_msg->range_min = config_.range_min_m;
  scan_msg->range_max = config_.range_max_m;

  scan_msg->ranges = snapshot.ranges;
  scan_msg->intensities = snapshot.intensities;

  scan_pub_->publish(std::move(scan_msg));

  // Update analyzers
  front_analyzer_->update(snapshot.ranges, config_.angle_resolution_deg, now);

  // Publish front distance
  auto front_analysis = front_analyzer_->get_analysis();
  auto front_msg = std::make_unique<std_msgs::msg::Float32>();
  front_msg->data = front_analysis.distance_m;
  front_pub_->publish(std::move(front_msg));

  // Publish closest point
  auto closest = closest_finder_->find(snapshot.ranges,
                                        config_.angle_resolution_deg);
  auto closest_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
  closest_msg->data.resize(2);
  closest_msg->data[0] = closest.distance_m;
  closest_msg->data[1] = closest.angle_deg;
  closest_pub_->publish(std::move(closest_msg));
}

void LD06Node::publish_front_immediate(float distance_m) {
  auto now = std::chrono::system_clock::now();
  auto time_since_last = std::chrono::duration_cast<std::chrono::duration<
      double>>(now - last_immediate_front_pub_);

  if (time_since_last.count() < config_.immediate_front_pub_min_period) {
    return;
  }

  last_immediate_front_pub_ = now;

  auto msg = std::make_unique<std_msgs::msg::Float32>();
  msg->data = distance_m;
  front_pub_->publish(std::move(msg));
}

float LD06Node::corrected_angle(float raw_angle_deg) const {
  float angle = config_.invert_angle_direction ? -raw_angle_deg :
                                                  raw_angle_deg;
  angle += config_.angle_offset_deg;
  return angle_utils::normalize_0_360(angle);
}

int LD06Node::angle_to_index(float angle_deg) const {
  int bin_count = static_cast<int>(std::round(360.0 /
                                               config_.angle_resolution_deg));
  return static_cast<int>(std::round(
             angle_utils::normalize_0_360(angle_deg) /
             config_.angle_resolution_deg)) %
         bin_count;
}

void LD06Node::log_startup(rclcpp::Logger logger, const Config& cfg) {
  RCLCPP_INFO(logger,
              "LD06 started on %s, baud=%u, offset=%.1f, invert=%s, "
              "front=[%.1f, %.1f]",
              cfg.port.c_str(), cfg.baudrate, cfg.angle_offset_deg,
              cfg.invert_angle_direction ? "true" : "false",
              cfg.front_min_deg, cfg.front_max_deg);
}

}  // namespace ld06

// ============================================================================
// ROS2 Main
// ============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ld06::LD06Node>());
  rclcpp::shutdown();
  return 0;
}
