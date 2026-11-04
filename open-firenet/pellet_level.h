#pragma once
#include "pellet_calc.h"
#include <mutex>
#include <string>

#ifdef ARDUINO
#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#if __has_include(<VL53L0X.h>)
#define OF_HAVE_VL53L0X 1
#include <VL53L0X.h>
#else
#define OF_HAVE_VL53L0X 0
#endif
#endif

namespace firenet {
namespace pellet {

class PelletManager {
 public:
  PelletManager() = default;

  void begin();
  void loop();

  PelletConfig config() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
  }

  PelletResult result() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_;
  }

  bool isAvailable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return result_.available && result_.valid;
  }

  uint32_t maxDurationMs() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return max_duration_ms_;
  }

  // Updates configuration and reconfigures sensor/bus if changed.
  bool updateConfig(const PelletConfig& new_cfg);

  // Generates JSON for /api/state: "pellet": { ... }
  std::string jsonSection() const;

  // Generates JSON for GET /api/pellet
  std::string jsonConfig() const;

#ifdef ARDUINO
  void loadFromNvs(Preferences& prefs);
  void saveToNvs(Preferences& prefs);
#endif

 private:
  void initSensor();
  void performMeasurement();

  mutable std::mutex mutex_;
  PelletConfig config_;
  PelletResult result_;
  PelletFilter filter_;

  uint32_t last_measure_ms_ = 0;
  uint32_t last_probe_ms_ = 0;
  uint32_t max_duration_ms_ = 0;
  bool wire_initialized_ = false;

#if defined(ARDUINO) && OF_HAVE_VL53L0X
  VL53L0X sensor_;
#endif
};

extern PelletManager g_pelletLevel;

}  // namespace pellet
}  // namespace firenet
