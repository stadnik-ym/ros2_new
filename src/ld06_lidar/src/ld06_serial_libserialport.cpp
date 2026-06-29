#include "../include/ld06_lidar/ld06_serial_libserialport.hpp"
#include "../include/ld06_lidar/ld06_node.hpp"

#include <cstring>
#include <chrono>
#include <thread>

namespace ld06 {

SerialReader::SerialReader(const std::string& port, uint32_t baudrate,
                           uint32_t timeout_ms) {
  struct sp_port* port_ptr = nullptr;

  // Отримати список портів
  sp_return error = sp_get_port_by_name(port.c_str(), &port_ptr);
  if (error != SP_OK) {
    throw std::runtime_error("Failed to get port: " + port);
  }

  if (!port_ptr) {
    throw std::runtime_error("Port not found: " + port);
  }

  port_ = port_ptr;

  // Відкрити порт
  error = sp_open(port_, SP_MODE_READ_WRITE);
  if (error != SP_OK) {
    throw_error("Failed to open port", error);
  }

  // Встановити параметри
  error = sp_set_baudrate(port_, baudrate);
  if (error != SP_OK) {
    sp_close(port_);
    throw_error("Failed to set baudrate", error);
  }

  // 8 bits, 1 stop bit, no parity
  error = sp_set_bits(port_, 8);
  if (error != SP_OK) {
    sp_close(port_);
    throw_error("Failed to set data bits", error);
  }

  error = sp_set_parity(port_, SP_PARITY_NONE);
  if (error != SP_OK) {
    sp_close(port_);
    throw_error("Failed to set parity", error);
  }

  error = sp_set_stopbits(port_, 1);
  if (error != SP_OK) {
    sp_close(port_);
    throw_error("Failed to set stop bits", error);
  }

  // Set flow control to none
  error = sp_set_flowcontrol(port_, SP_FLOWCONTROL_NONE);
  if (error != SP_OK) {
    sp_close(port_);
    throw_error("Failed to set flow control", error);
  }

  // Note: timeout буде встановлено в sp_blocking_read() функції
  // libserialport не має sp_set_read_blocking_timeout - timeout передається у функцію читання
}

SerialReader::~SerialReader() {
  if (port_ != nullptr) {
    sp_close(port_);
    sp_free_port(port_);
    port_ = nullptr;
  }
}

void SerialReader::throw_error(const std::string& msg, int error_code) {
  const char* error_str = sp_last_error_message();
  std::string full_msg = msg + ": " + (error_str ? error_str : "unknown error");
  sp_free_error_message(const_cast<char*>(error_str));
  throw std::runtime_error(full_msg);
}

bool SerialReader::sync_to_header() {
  while (buffer_.size() > 0) {
    if (buffer_[0] == PACKET_HEADER_0) {
      if (buffer_.size() > 1 && buffer_[1] == PACKET_HEADER_1) {
        return true;
      }
      if (buffer_.size() == 1) {
        return false;  // Wait for next byte
      }
    }
    buffer_.erase(buffer_.begin());
  }
  return false;
}

bool SerialReader::read_packet(std::vector<uint8_t>& packet) {
  packet.clear();

  if (!is_open()) {
    return false;
  }

  // Read available data (non-blocking check)
  unsigned int bytes_waiting = 0;
  sp_return error = sp_input_waiting(port_);
  if (error > 0) {
    bytes_waiting = static_cast<unsigned int>(error);
  }

  if (bytes_waiting > 0) {
    unsigned int to_read = std::min(bytes_waiting, 100u - (unsigned int)buffer_.size());
    if (to_read > 0) {
      std::vector<uint8_t> temp(to_read);
      error = sp_nonblocking_read(port_, temp.data(), to_read);
      if (error > 0) {
        buffer_.insert(buffer_.end(), temp.begin(), temp.begin() + error);
      }
    }
  }

  // Try to sync to header
  if (!sync_to_header()) {
    if (buffer_.empty()) {
      // Block-read one byte to avoid spinning
      // Timeout 5ms for single byte
      std::vector<uint8_t> temp(1);
      int bytes_read = sp_blocking_read(port_, temp.data(), 1, 5);
      if (bytes_read > 0) {
        buffer_.push_back(temp[0]);
        return sync_to_header();
      }
    }
    return false;
  }

  // Wait for full packet
  if (buffer_.size() < PACKET_SIZE) {
    unsigned int needed = PACKET_SIZE - buffer_.size();
    std::vector<uint8_t> temp(needed);

    // Timeout 50ms for remaining bytes
    int bytes_read = sp_blocking_read(port_, temp.data(), needed, 50);
    if (bytes_read > 0) {
      buffer_.insert(buffer_.end(), temp.begin(), temp.begin() + bytes_read);
    }
  }

  if (buffer_.size() >= PACKET_SIZE) {
    packet.assign(buffer_.begin(), buffer_.begin() + PACKET_SIZE);
    buffer_.erase(buffer_.begin(), buffer_.begin() + PACKET_SIZE);
    return true;
  }

  return false;
}

}  // namespace ld06
