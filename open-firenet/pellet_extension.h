#pragma once
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>
#include "firenet_extension.h"
#include "pellet_calc.h"

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#if __has_include(<VL53L0X.h>)
#define OF_HAVE_VL53L0X 1
#include <VL53L0X.h>
#else
#define OF_HAVE_VL53L0X 0
#endif
#endif

namespace firenet {

class PelletExtension : public Extension {
 public:
  PelletExtension() {
    initValues();
  }

  const char* id() const override { return "pellet_level"; }
  const char* name() const override { return "Pellet Level Sensor"; }
  uint8_t i2cAddress() const override { return 0x29; }

  size_t configFieldCount() const override { return 4; }

  const ConfigField* configFields() const override {
    static const ConfigField fields[] = {
      {"tank_height_cm", "Tank height", ConfigField::FLOAT, "65.0", "cm", 10.0f, 200.0f},
      {"min_distance_cm", "Min distance / sensor offset", ConfigField::FLOAT, "10.3", "cm", 1.0f, 50.0f},
      {"capacity_kg", "Tank capacity", ConfigField::FLOAT, "40.0", "kg", 5.0f, 200.0f},
      {"low_threshold_percent", "Low pellet alert threshold", ConfigField::FLOAT, "20.0", "%", 0.0f, 100.0f}
    };
    return fields;
  }

  bool begin(const std::map<std::string, std::string>& config, I2CBus& bus, std::string* error_msg) override {
    bus_ = &bus;
    parseConfig(config);

    if (!bus_->isReady()) {
      if (error_msg) *error_msg = "I2C bus not ready";
      running_ = false;
      healthy_ = false;
      status_ = "I2C bus not ready";
      return false;
    }

    if (!bus_->probe(i2cAddress())) {
      if (error_msg) *error_msg = "VL53L0X not found at I2C address 0x29";
      running_ = false;
      healthy_ = false;
      status_ = "Sensor not found (0x29)";
      return false;
    }

#if defined(ARDUINO) && OF_HAVE_VL53L0X
    sensor_.setBus(&Wire);
    sensor_.setTimeout(100);
    if (!sensor_.init()) {
      if (error_msg) *error_msg = "VL53L0X init() failed";
      running_ = false;
      healthy_ = false;
      status_ = "Sensor init failed";
      return false;
    }
    sensor_.setMeasurementTimingBudget(50000); // 50 ms
#endif

    running_ = true;
    healthy_ = true;
    error_count_ = 0;
    status_ = "ok";
    last_poll_ms_ = 0;
    filter_.clear();

    // Trigger initial measurement
    pollMeasurement();
    return true;
  }

  void stop() override {
    running_ = false;
    healthy_ = true;
    status_ = "stopped";
    filter_.clear();
    for (auto& v : values_) {
      v.valid = false;
    }
  }

  void poll() override {
    if (!running_) return;

#ifdef ARDUINO
    uint32_t now = millis();
    if (last_poll_ms_ != 0 && (now - last_poll_ms_ < 10000)) {
      return; // poll every 10 s
    }
    last_poll_ms_ = now;
#endif

    pollMeasurement();
  }

  bool isRunning() const override { return running_; }
  bool isHealthy() const override { return healthy_; }
  const char* statusMessage() const override { return status_.c_str(); }

  size_t valueCount() const override { return values_.size(); }
  const ExtensionValue* values() const override { return values_.data(); }

  // Expose parsed PelletConfig and result for backwards-compatibility
  pellet::PelletConfig pelletConfig() const { return cfg_; }
  pellet::PelletResult pelletResult() const { return result_; }

  std::string legacyJsonSection() const {
    if (!running_ || !healthy_ || !result_.valid) {
      return "\"pellet\":{\"available\":false,\"distance_cm\":null,\"level_percent\":null,\"remaining_kg\":null,\"low\":null}";
    }
    char buf[160];
    snprintf(buf, sizeof(buf),
      "\"pellet\":{\"available\":true,\"distance_cm\":%.1f,\"level_percent\":%.1f,\"remaining_kg\":%.1f,\"low\":%s}",
      result_.distance_cm,
      result_.level_percent,
      result_.remaining_kg,
      result_.low ? "true" : "false"
    );
    return std::string(buf);
  }

 private:
  void initValues() {
    values_ = {
      {"distance_cm", "Distance", ExtensionValue::DISTANCE, "cm", false, 0.0f, false, false},
      {"level_percent", "Level", ExtensionValue::PERCENT, "%", false, 0.0f, false, false},
      {"remaining_kg", "Remaining", ExtensionValue::MASS, "kg", false, 0.0f, false, false},
      {"low", "Low Alert", ExtensionValue::BINARY_ALERT, "", true, 0.0f, false, false}
    };
  }

  void parseConfig(const std::map<std::string, std::string>& config) {
    auto get_f = [&](const char* key, float def) -> float {
      auto it = config.find(key);
      if (it != config.end() && !it->second.empty()) {
        return std::strtof(it->second.c_str(), nullptr);
      }
      return def;
    };
    cfg_.enabled = true;
    cfg_.tank_height_cm = get_f("tank_height_cm", 65.0f);
    cfg_.min_distance_cm = get_f("min_distance_cm", 10.3f);
    cfg_.capacity_kg = get_f("capacity_kg", 40.0f);
    cfg_.low_threshold_percent = get_f("low_threshold_percent", 20.0f);
  }

  void pollMeasurement() {
#if defined(ARDUINO) && OF_HAVE_VL53L0X
    if (!bus_ || !bus_->isReady()) {
      handleError("I2C bus not ready");
      return;
    }

    uint16_t dist_mm = 0;
    bool timeout = false;
    {
      std::lock_guard<std::mutex> lk(bus_->mutex());
      dist_mm = sensor_.readRangeSingleMillimeters();
      timeout = sensor_.timeoutOccurred();
    }

    bool invalid = timeout || dist_mm >= 8190 || dist_mm == 65535 || dist_mm > 2000;
    if (invalid) {
      handleError("Read timeout or out of range");
      return;
    }

    // Success
    error_count_ = 0;
    healthy_ = true;
    status_ = "ok";
    filter_.recordSuccess(dist_mm / 10.0f);
    result_ = pellet::calculatePelletLevel(filter_.averageDistance(), cfg_);
    updateValuesFrom(result_);
#else
    // Host / simulation measurement
    result_ = pellet::calculatePelletLevel(35.0f, cfg_);
    updateValuesFrom(result_);
#endif
  }

  void handleError(const char* reason) {
    error_count_++;
    if (error_count_ >= 5) {
      healthy_ = false;
      running_ = false;
      status_ = std::string("Failsafe: ") + reason;
      for (auto& v : values_) v.valid = false;
    } else {
      status_ = reason;
    }
  }

  void updateValuesFrom(const pellet::PelletResult& res) {
    values_[0].num_val = res.distance_cm;
    values_[0].valid = res.valid;

    values_[1].num_val = res.level_percent;
    values_[1].valid = res.valid;

    values_[2].num_val = res.remaining_kg;
    values_[2].valid = res.valid;

    values_[3].bool_val = res.low;
    values_[3].valid = res.valid;
  }

  I2CBus* bus_ = nullptr;
  pellet::PelletConfig cfg_;
  pellet::PelletResult result_;
  pellet::PelletFilter filter_;
  std::vector<ExtensionValue> values_;

  bool running_ = false;
  bool healthy_ = true;
  std::string status_ = "not started";
  int error_count_ = 0;
  uint32_t last_poll_ms_ = 0;

#if defined(ARDUINO) && OF_HAVE_VL53L0X
  VL53L0X sensor_;
#endif
};

} // namespace firenet
