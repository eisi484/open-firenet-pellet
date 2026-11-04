#include "pellet_level.h"
#include <cstdio>

namespace firenet {
namespace pellet {

PelletManager g_pelletLevel;

#if defined(ARDUINO)
static void pelletTaskFunc(void* arg) {
  auto* mgr = static_cast<PelletManager*>(arg);
  for (;;) {
    mgr->loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#endif

void PelletManager::begin() {
#if defined(ARDUINO)
  xTaskCreatePinnedToCore(pelletTaskFunc, "pellet", 4096, this, 1, nullptr, 1);
#endif
}

bool PelletManager::updateConfig(const PelletConfig& new_cfg) {
  if (!new_cfg.isValid()) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  bool pins_changed = (new_cfg.sda_pin != config_.sda_pin || new_cfg.scl_pin != config_.scl_pin);
  bool enabled_changed = (new_cfg.enabled != config_.enabled);
  config_ = new_cfg;

  if (enabled_changed || pins_changed) {
    wire_initialized_ = false;
    filter_.clear();
    result_.available = false;
    result_.valid = false;
    last_probe_ms_ = 0;
  }
  return true;
}

void PelletManager::initSensor() {
#if defined(ARDUINO) && OF_HAVE_VL53L0X
  if (!config_.enabled) return;

  if (!wire_initialized_) {
    Wire.begin(config_.sda_pin, config_.scl_pin, 100000);
    Wire.setTimeOut(50);
    wire_initialized_ = true;
  }

  Wire.beginTransmission(0x29);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    sensor_.setBus(&Wire);
    sensor_.setTimeout(100);
    if (sensor_.init()) {
      sensor_.setMeasurementTimingBudget(50000); // 50 ms budget
      std::lock_guard<std::mutex> lock(mutex_);
      result_.available = true;
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  result_.available = false;
  result_.valid = false;
#endif
}

void PelletManager::performMeasurement() {
#if defined(ARDUINO) && OF_HAVE_VL53L0X
  if (!config_.enabled || !result_.available) return;

  uint32_t t0 = millis();
  uint16_t dist_mm = sensor_.readRangeSingleMillimeters();
  uint32_t duration = millis() - t0;

  std::lock_guard<std::mutex> lock(mutex_);
  if (duration > max_duration_ms_) {
    max_duration_ms_ = duration;
  }

  bool timeout = sensor_.timeoutOccurred();
  bool invalid = timeout || dist_mm >= 8190 || dist_mm == 65535 || dist_mm > 2000;

  if (invalid) {
    bool reset = filter_.recordError();
    if (reset) {
      result_.valid = false;
      result_.available = false;  // trigger probe retry after 60s
    }
  } else {
    filter_.recordSuccess(dist_mm / 10.0f);
    result_ = calculatePelletLevel(filter_.averageDistance(), config_);
    result_.available = true;
    result_.valid = true;
  }
#endif
}

void PelletManager::loop() {
#if defined(ARDUINO)
  uint32_t now = millis();

  PelletConfig cfg;
  bool is_avail = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cfg = config_;
    is_avail = result_.available;
  }

  if (!cfg.enabled) {
    return;
  }

  // If sensor is not available, probe address 0x29 every 60 s
  if (!is_avail) {
    if (last_probe_ms_ == 0 || (now - last_probe_ms_ >= 60000)) {
      last_probe_ms_ = now;
      initSensor();
    }
    return;
  }

  // If sensor is available, measure every 10 s
  if (last_measure_ms_ == 0 || (now - last_measure_ms_ >= 10000)) {
    last_measure_ms_ = now;
    performMeasurement();
  }
#endif
}

std::string PelletManager::jsonSection() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!config_.enabled || !result_.available || !result_.valid) {
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

std::string PelletManager::jsonConfig() const {
  std::lock_guard<std::mutex> lock(mutex_);
  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"enabled\":%s,\"sda_pin\":%d,\"scl_pin\":%d,\"tank_height_cm\":%.1f,\"min_distance_cm\":%.1f,\"capacity_kg\":%.1f,\"low_threshold_percent\":%.1f,\"max_duration_ms\":%u}",
    config_.enabled ? "true" : "false",
    config_.sda_pin,
    config_.scl_pin,
    config_.tank_height_cm,
    config_.min_distance_cm,
    config_.capacity_kg,
    config_.low_threshold_percent,
    (unsigned)max_duration_ms_
  );
  return std::string(buf);
}

#ifdef ARDUINO
void PelletManager::loadFromNvs(Preferences& prefs) {
  PelletConfig cfg;
  prefs.begin("firenet", true);
  cfg.enabled = prefs.getBool("pel_en", false);
  cfg.sda_pin = prefs.getInt("pel_sda", 9);
  cfg.scl_pin = prefs.getInt("pel_scl", 18);
  cfg.tank_height_cm = prefs.getFloat("pel_h", 70.0f);
  cfg.min_distance_cm = prefs.getFloat("pel_min", 0.5f);
  cfg.capacity_kg = prefs.getFloat("pel_kg", 45.0f);
  cfg.low_threshold_percent = prefs.getFloat("pel_low", 20.0f);
  prefs.end();

  if (cfg.isValid()) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = cfg;
  }
}

void PelletManager::saveToNvs(Preferences& prefs) {
  std::lock_guard<std::mutex> lock(mutex_);
  prefs.begin("firenet", false);
  prefs.putBool("pel_en", config_.enabled);
  prefs.putInt("pel_sda", config_.sda_pin);
  prefs.putInt("pel_scl", config_.scl_pin);
  prefs.putFloat("pel_h", config_.tank_height_cm);
  prefs.putFloat("pel_min", config_.min_distance_cm);
  prefs.putFloat("pel_kg", config_.capacity_kg);
  prefs.putFloat("pel_low", config_.low_threshold_percent);
  prefs.end();
}
#endif

}  // namespace pellet
}  // namespace firenet
