#pragma once

#include <libserialport.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstdint>

namespace ld06 {

// ============================================================================
// Serial Reader для libserialport
// ============================================================================

class SerialReader {
 public:
  SerialReader(const std::string& port, uint32_t baudrate,
               uint32_t timeout_ms = 5);
  ~SerialReader();

  bool is_open() const { return port_ != nullptr; }
  bool read_packet(std::vector<uint8_t>& packet);

 private:
  struct sp_port* port_ = nullptr;
  std::vector<uint8_t> buffer_;

  bool sync_to_header();
  void throw_error(const std::string& msg, int error_code);
};

}  // namespace ld06
