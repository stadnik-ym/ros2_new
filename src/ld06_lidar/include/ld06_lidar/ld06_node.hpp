#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <deque>

namespace ld06 {

// ============================================================================
// Constants
// ============================================================================

constexpr int PACKET_SIZE = 47;
constexpr uint8_t PACKET_HEADER_0 = 0x54;
constexpr uint8_t PACKET_HEADER_1 = 0x2C;
constexpr int POINTS_PER_PACKET = 12;
constexpr int DEFAULT_BIN_COUNT = 360;  // 1 degree resolution

constexpr float MM_TO_M = 0.001f;

// ============================================================================
// CRC Calculator
// ============================================================================

class CRC8Calculator {
 public:
  CRC8Calculator();
  uint8_t calculate(const uint8_t* data, size_t len) const;

 private:
  std::array<uint8_t, 256> table_;
};

// ============================================================================
// Packet Parser
// ============================================================================

struct LaserPoint {
  float distance_m;
  uint8_t confidence;
  float raw_angle_deg;
};

struct PacketData {
  float start_angle_deg;
  float end_angle_deg;
  std::vector<LaserPoint> points;
  std::chrono::system_clock::time_point timestamp;
};

class PacketParser {
 public:
  explicit PacketParser(const CRC8Calculator& crc_calc);

  bool is_valid_packet(const uint8_t* data, size_t len) const;
  std::optional<PacketData> parse(const uint8_t* data, size_t len);

 private:
  const CRC8Calculator& crc_;
};

// ============================================================================
// Angle Utilities
// ============================================================================

namespace angle_utils {
  float normalize_0_360(float angle_deg);
  float normalize_180(float angle_deg);
}  // namespace angle_utils

// ============================================================================
// Scan Data Container
// ============================================================================

struct ScanSnapshot {
  std::vector<float> ranges;
  std::vector<float> intensities;
  std::chrono::system_clock::time_point timestamp;
};

class ScanBuffer {
 public:
  explicit ScanBuffer(int bin_count);

  void update_bin(int idx, float distance, uint8_t intensity,
                  const std::chrono::system_clock::time_point& now);
  ScanSnapshot get_snapshot(
      float point_max_age_sec,
      const std::chrono::system_clock::time_point& now) const;

  void clear();

 private:
  std::vector<float> ranges_;
  std::vector<float> intensities_;
  std::vector<std::chrono::system_clock::time_point> update_times_;
  mutable std::mutex lock_;
};

// ============================================================================
// Front Sector Analysis
// ============================================================================

struct FrontAnalysis {
  float distance_m;
  int valid_points;
};

class FrontSectorAnalyzer {
 public:
  FrontSectorAnalyzer(float min_deg, float max_deg, int filter_window,
                      float hold_sec, int min_points);

  void update(const std::vector<float>& ranges, float angle_resolution_deg,
              const std::chrono::system_clock::time_point& now);
  FrontAnalysis get_analysis() const;

  std::optional<float> get_last_valid() const;
  void reset();

  // ✅ PUBLIC - потрібна для read_loop()
  bool angle_in_front(float angle_deg) const;

 private:
  float min_deg_;
  float max_deg_;
  int filter_window_;
  float hold_sec_;
  int min_points_;

  std::deque<float> history_;
  float last_valid_distance_;
  std::chrono::system_clock::time_point last_valid_time_;
  mutable std::mutex lock_;
};

// ============================================================================
// Closest Point Finder
// ============================================================================

struct ClosestPoint {
  float distance_m;
  float angle_deg;
  bool valid;
};

class ClosestPointFinder {
 public:
  ClosestPoint find(const std::vector<float>& ranges,
                    float angle_resolution_deg) const;
};

}  // namespace ld06
