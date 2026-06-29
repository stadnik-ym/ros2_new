#include "../include/ld06_lidar/ld06_node.hpp"

#include <cstring>
#include <algorithm>

namespace ld06 {

// ============================================================================
// CRC8Calculator Implementation
// ============================================================================

// clang-format off
constexpr std::array<uint8_t, 256> CRC_TABLE_DATA = {{
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae,
    0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1,
    0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f,
    0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2,
    0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64,
    0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec,
    0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3,
    0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a,
    0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
}};
// clang-format on

CRC8Calculator::CRC8Calculator() : table_(CRC_TABLE_DATA) {}

uint8_t CRC8Calculator::calculate(const uint8_t* data, size_t len) const {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = table_[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

// ============================================================================
// PacketParser Implementation
// ============================================================================

PacketParser::PacketParser(const CRC8Calculator& crc_calc) : crc_(crc_calc) {}

bool PacketParser::is_valid_packet(const uint8_t* data, size_t len) const {
  if (len != PACKET_SIZE) return false;
  if (data[0] != PACKET_HEADER_0 || data[1] != PACKET_HEADER_1) return false;

  uint8_t expected_crc = crc_.calculate(data, PACKET_SIZE - 1);
  return expected_crc == data[PACKET_SIZE - 1];
}

std::optional<PacketData> PacketParser::parse(const uint8_t* data,
                                               size_t len) {
  if (!is_valid_packet(data, len)) {
    return std::nullopt;
  }

  PacketData packet;
  packet.timestamp = std::chrono::system_clock::now();

  // Extract angles (little-endian uint16 / 100.0)
  uint16_t start_raw = (data[5] << 8) | data[4];
  uint16_t end_raw = (data[43] << 8) | data[42];

  packet.start_angle_deg = start_raw / 100.0f;
  packet.end_angle_deg = end_raw / 100.0f;

  float angle_span = packet.end_angle_deg - packet.start_angle_deg;
  if (angle_span < -180.0f) {
    angle_span += 360.0f;
  } else if (angle_span > 180.0f) {
    angle_span -= 360.0f;
  }

  float angle_step = angle_span / (POINTS_PER_PACKET - 1);

  // Parse 12 points
  packet.points.reserve(POINTS_PER_PACKET);
  for (int i = 0; i < POINTS_PER_PACKET; ++i) {
    size_t offset = 6 + i * 3;

    uint16_t dist_mm = (data[offset + 1] << 8) | data[offset];
    uint8_t confidence = data[offset + 2];

    LaserPoint point;
    point.distance_m = dist_mm * MM_TO_M;
    point.confidence = confidence;
    point.raw_angle_deg = packet.start_angle_deg + angle_step * i;

    packet.points.push_back(point);
  }

  return packet;
}

// ============================================================================
// Angle Utilities Implementation
// ============================================================================

namespace angle_utils {

float normalize_0_360(float angle_deg) {
  angle_deg = std::fmod(angle_deg, 360.0f);
  if (angle_deg < 0.0f) {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

float normalize_180(float angle_deg) {
  return std::fmod(angle_deg + 180.0f, 360.0f) - 180.0f;
}

}  // namespace angle_utils

// ============================================================================
// ScanBuffer Implementation
// ============================================================================

ScanBuffer::ScanBuffer(int bin_count)
    : ranges_(bin_count, INFINITY),
      intensities_(bin_count, 0.0f),
      update_times_(bin_count) {}

void ScanBuffer::update_bin(
    int idx, float distance, uint8_t intensity,
    const std::chrono::system_clock::time_point& now) {
  std::lock_guard<std::mutex> lock(lock_);

  if (idx < 0 || idx >= static_cast<int>(ranges_.size())) {
    return;
  }

  const auto& prev_time = update_times_[idx];
  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - prev_time);

  if (age.count() > 50) {
    // Data is old, replace
    ranges_[idx] = distance;
    intensities_[idx] = intensity;
    update_times_[idx] = now;
  } else if (distance < ranges_[idx]) {
    // Data is recent, take minimum
    ranges_[idx] = distance;
    intensities_[idx] = intensity;
    update_times_[idx] = now;
  }
}

ScanSnapshot ScanBuffer::get_snapshot(
    float point_max_age_sec,
    const std::chrono::system_clock::time_point& now) const {
  std::lock_guard<std::mutex> lock(lock_);

  ScanSnapshot snapshot;
  snapshot.ranges = ranges_;
  snapshot.intensities = intensities_;
  snapshot.timestamp = now;

  // Mark stale data as invalid
  for (size_t i = 0; i < ranges_.size(); ++i) {
    if (update_times_[i].time_since_epoch().count() == 0) {
      snapshot.ranges[i] = INFINITY;
      snapshot.intensities[i] = 0.0f;
    } else {
      auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - update_times_[i]);
      if (age.count() > static_cast<int>(point_max_age_sec * 1000)) {
        snapshot.ranges[i] = INFINITY;
        snapshot.intensities[i] = 0.0f;
      }
    }
  }

  return snapshot;
}

void ScanBuffer::clear() {
  std::lock_guard<std::mutex> lock(lock_);
  std::fill(ranges_.begin(), ranges_.end(), INFINITY);
  std::fill(intensities_.begin(), intensities_.end(), 0.0f);
  std::fill(update_times_.begin(), update_times_.end(),
            std::chrono::system_clock::time_point{});
}

// ============================================================================
// FrontSectorAnalyzer Implementation
// ============================================================================

FrontSectorAnalyzer::FrontSectorAnalyzer(float min_deg, float max_deg,
                                         int filter_window, float hold_sec,
                                         int min_points)
    : min_deg_(min_deg),
      max_deg_(max_deg),
      filter_window_(filter_window),
      hold_sec_(hold_sec),
      min_points_(min_points),
      last_valid_distance_(-1.0f) {}

bool FrontSectorAnalyzer::angle_in_front(float angle_deg) const {
  angle_deg = angle_utils::normalize_180(angle_deg);

  if (min_deg_ <= max_deg_) {
    return min_deg_ <= angle_deg && angle_deg <= max_deg_;
  }
  return angle_deg >= min_deg_ || angle_deg <= max_deg_;
}

void FrontSectorAnalyzer::update(const std::vector<float>& ranges,
                                  float angle_resolution_deg,
                                  const std::chrono::system_clock::time_point& now) {
  std::vector<float> front_values;

  for (size_t idx = 0; idx < ranges.size(); ++idx) {
    if (std::isinf(ranges[idx]) || std::isnan(ranges[idx])) {
      continue;
    }

    float angle_0_360 = idx * angle_resolution_deg;
    if (angle_in_front(angle_0_360)) {
      front_values.push_back(ranges[idx]);
    }
  }

  std::lock_guard<std::mutex> lock(lock_);

  if (static_cast<int>(front_values.size()) >= min_points_) {
    std::sort(front_values.begin(), front_values.end());
    float instant_min = front_values[0];

    history_.push_back(instant_min);
    if (static_cast<int>(history_.size()) > filter_window_) {
      history_.pop_front();
    }

    float filtered_min = *std::min_element(history_.begin(), history_.end());
    last_valid_distance_ = filtered_min;
    last_valid_time_ = now;
  } else {
    // No points in front now
    if (last_valid_time_.time_since_epoch().count() > 0) {
      auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_valid_time_);
      if (age.count() > static_cast<int>(hold_sec_ * 1000)) {
        last_valid_distance_ = -1.0f;
      }
    }
  }
}

FrontAnalysis FrontSectorAnalyzer::get_analysis() const {
  std::lock_guard<std::mutex> lock(lock_);
  FrontAnalysis analysis;
  analysis.distance_m = last_valid_distance_;
  analysis.valid_points = static_cast<int>(history_.size());
  return analysis;
}

std::optional<float> FrontSectorAnalyzer::get_last_valid() const {
  std::lock_guard<std::mutex> lock(lock_);
  if (last_valid_distance_ > 0.0f) {
    return last_valid_distance_;
  }
  return std::nullopt;
}

void FrontSectorAnalyzer::reset() {
  std::lock_guard<std::mutex> lock(lock_);
  history_.clear();
  last_valid_distance_ = -1.0f;
  last_valid_time_ = std::chrono::system_clock::time_point{};
}

// ============================================================================
// ClosestPointFinder Implementation
// ============================================================================

ClosestPoint ClosestPointFinder::find(const std::vector<float>& ranges,
                                       float angle_resolution_deg) const {
  float best_dist = INFINITY;
  float best_angle = 0.0f;
  bool found = false;

  for (size_t idx = 0; idx < ranges.size(); ++idx) {
    if (!std::isinf(ranges[idx]) && !std::isnan(ranges[idx])) {
      if (ranges[idx] < best_dist) {
        best_dist = ranges[idx];
        float angle_0_360 = idx * angle_resolution_deg;
        best_angle = angle_utils::normalize_180(angle_0_360);
        found = true;
      }
    }
  }

  return ClosestPoint{best_dist, best_angle, found};
}

}  // namespace ld06
