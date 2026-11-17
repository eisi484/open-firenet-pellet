#pragma once
#include <cstdint>
#include <mutex>
#include <string>

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

namespace firenet {

class I2CBus {
 public:
  I2CBus() = default;

  ~I2CBus() {
    end();
  }

  bool begin(int sda, int scl, uint32_t frequency = 100000) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ready_ && sda_ == sda && scl_ == scl) {
      return true;
    }
    endUnlocked();

    sda_ = sda;
    scl_ = scl;
    freq_ = frequency;

#ifdef ARDUINO
    bool ok = Wire.begin(sda, scl, frequency);
    Wire.setTimeOut(50); // 50ms I2C timeout
    ready_ = ok;
    return ok;
#else
    ready_ = (sda >= 0 && scl >= 0 && sda != scl);
    return ready_;
#endif
  }

  void end() {
    std::lock_guard<std::mutex> lock(mutex_);
    endUnlocked();
  }

  bool isReady() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ready_;
  }

  int sda() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sda_;
  }

  int scl() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return scl_;
  }

  // Probes an I2C address by sending a write transaction with no data.
  // Returns true if ACK received, false on NACK/timeout.
  bool probe(uint8_t address) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_) return false;
#ifdef ARDUINO
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
#else
    return mock_probe_result_;
#endif
  }

  std::mutex& mutex() { return mutex_; }

#ifndef ARDUINO
  void setMockProbeResult(bool ok) { mock_probe_result_ = ok; }
#endif

 private:
  void endUnlocked() {
#ifdef ARDUINO
    if (ready_) {
      Wire.end();
    }
#endif
    ready_ = false;
  }

  mutable std::mutex mutex_;
  int sda_ = -1;
  int scl_ = -1;
  uint32_t freq_ = 100000;
  bool ready_ = false;
#ifndef ARDUINO
  bool mock_probe_result_ = true;
#endif
};

} // namespace firenet
