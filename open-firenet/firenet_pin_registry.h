#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace firenet {

class PinRegistry {
 public:
  PinRegistry() {
    // Reserve system-critical pins on ESP32-S3
    reserveHardwired(0, "boot_button");
    reserveHardwired(19, "usb_cdc_dn"); // USB D- for stove link
    reserveHardwired(20, "usb_cdc_dp"); // USB D+ for stove link
    
    // SPI Flash pins (ESP32-S3 default: GPIO 26-32)
    for (int p = 26; p <= 32; ++p) {
      reserveHardwired(p, "spi_flash");
    }

    // Default UART0 / debug pins
    reserveHardwired(43, "uart0_tx");
    reserveHardwired(44, "uart0_rx");
  }

  void reserveHardwired(int pin, const char* owner) {
    if (pin >= 0 && pin < 49) {
      hardwired_[pin] = owner;
    }
  }

  void reserveBoardDisplay(const std::string& board) {
    if (board == "pocket_dongle") {
      // Pocket Dongle display pins: 10, 11, 12, 13, 14
      reserve(10, "display");
      reserve(11, "display");
      reserve(12, "display");
      reserve(13, "display");
      reserve(14, "display");
    } else if (board == "tdongle") {
      // T-Dongle display pins: 5, 3, 4, 2, 1, 38
      reserve(5, "display");
      reserve(3, "display");
      reserve(4, "display");
      reserve(2, "display");
      reserve(1, "display");
      reserve(38, "display");
    }
  }

  void reserveBoardEthernet(const std::string& board) {
    if (board == "waveshare_s3_eth") {
      // Waveshare ESP32-S3-ETH: CS=14, INT=10, RST=9, SCLK=13, MISO=12, MOSI=11
      reserve(14, "ethernet");
      reserve(10, "ethernet");
      reserve(9, "ethernet");
      reserve(13, "ethernet");
      reserve(12, "ethernet");
      reserve(11, "ethernet");
    }
  }

  bool isAvailable(int pin, std::string* reason = nullptr) const {
    if (pin < 0 || pin > 48) {
      if (reason) *reason = "Invalid GPIO pin (must be 0..48)";
      return false;
    }
    auto it_h = hardwired_.find(pin);
    if (it_h != hardwired_.end()) {
      if (reason) *reason = std::string("Pin ") + std::to_string(pin) + " is reserved by system (" + it_h->second + ")";
      return false;
    }
    auto it_d = dynamic_.find(pin);
    if (it_d != dynamic_.end()) {
      if (reason) *reason = std::string("Pin ") + std::to_string(pin) + " is already in use by " + it_d->second;
      return false;
    }
    return true;
  }

  bool reserve(int pin, const char* owner, std::string* reason = nullptr) {
    if (!isAvailable(pin, reason)) {
      return false;
    }
    dynamic_[pin] = owner;
    return true;
  }

  void release(int pin) {
    dynamic_.erase(pin);
  }

  void releaseByOwner(const char* owner) {
    for (auto it = dynamic_.begin(); it != dynamic_.end(); ) {
      if (it->second == owner) {
        it = dynamic_.erase(it);
      } else {
        ++it;
      }
    }
  }

  const char* getOwner(int pin) const {
    auto it_h = hardwired_.find(pin);
    if (it_h != hardwired_.end()) return it_h->second.c_str();
    auto it_d = dynamic_.find(pin);
    if (it_d != dynamic_.end()) return it_d->second.c_str();
    return nullptr;
  }

 private:
  std::unordered_map<int, std::string> hardwired_;
  std::unordered_map<int, std::string> dynamic_;
};

} // namespace firenet
