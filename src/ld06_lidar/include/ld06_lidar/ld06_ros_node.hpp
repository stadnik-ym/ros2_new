#pragma once

#include "ld06_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "ld06_serial_libserialport.hpp"

#include <queue>
#include <memory>

namespace ld06 {

// ============================================================================
// LD06 ROS2 Node
// ============================================================================

class LD06Node : public rclcpp::Node {
 public:
  LD06Node();
  ~LD06Node() override;

 private:
  // ---- Parameters ----
  struct Config {
    std::string port;
    uint32_t baudrate;
    std::string frame_id;

    std::string scan_topic;
    std::string front_distance_topic;
    std::string closest_topic;

    float publish_rate_hz;
    float range_min_m;
    float range_max_m;
    int min_confidence;

    float angle_resolution_deg;
    float angle_offset_deg;
    bool invert_angle_direction;

    float front_min_deg;
    float front_max_deg;
    float point_max_age_sec;
    int front_min_points;

    float front_hold_sec;
    int front_filter_window;

    bool immediate_front_publish;
    float immediate_front_pub_min_period;
  } config_;

  // ---- Publishers ----
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr closest_pub_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ---- Processing Components ----
  std::unique_ptr<CRC8Calculator> crc_calc_;
  std::unique_ptr<PacketParser> parser_;
  std::unique_ptr<ScanBuffer> scan_buffer_;
  std::unique_ptr<FrontSectorAnalyzer> front_analyzer_;
  std::unique_ptr<ClosestPointFinder> closest_finder_;
  std::unique_ptr<SerialReader> serial_reader_;

  // ---- Threading ----
  std::thread read_thread_;
  std::atomic<bool> running_{true};
  std::chrono::system_clock::time_point last_immediate_front_pub_;

  // ---- Methods ----
  void declare_parameters();
  void load_parameters();
  void init_publishers();
  void init_processing_components();
  void start_read_thread();

  void read_loop();
  void publish_data();
  void publish_front_immediate(float distance_m);

  float corrected_angle(float raw_angle_deg) const;
  int angle_to_index(float angle_deg) const;

  static void log_startup(rclcpp::Logger logger, const Config& cfg);
};

}  // namespace ld06
