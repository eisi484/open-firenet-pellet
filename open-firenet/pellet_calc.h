#pragma once
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <deque>
#include <numeric>

namespace firenet {
namespace pellet {

struct PelletConfig {
  bool enabled = false;
  int sda_pin = 9;
  int scl_pin = 18;
  float tank_height_cm = 70.0f;
  float min_distance_cm = 0.5f;
  float capacity_kg = 45.0f;
  float low_threshold_percent = 20.0f;

  bool isValid() const {
    if (sda_pin < 0 || scl_pin < 0) return false;
    // Don't allow using USB OTG pins (GPIO19/20) or Boot (GPIO0)
    if (sda_pin == 0 || sda_pin == 19 || sda_pin == 20) return false;
    if (scl_pin == 0 || scl_pin == 19 || scl_pin == 20) return false;
    if (sda_pin == scl_pin) return false;
    if (min_distance_cm >= tank_height_cm) return false;
    if (tank_height_cm <= 0.0f) return false;
    if (capacity_kg <= 0.0f) return false;
    if (low_threshold_percent < 0.0f || low_threshold_percent > 100.0f) return false;
    return true;
  }
};

struct PelletResult {
  bool available = false;
  bool valid = false;
  float distance_cm = 0.0f;
  float level_percent = 0.0f;
  float remaining_kg = 0.0f;
  bool low = false;
};

// Pure calculation function without hardware dependencies.
inline PelletResult calculatePelletLevel(float distance_cm, const PelletConfig& cfg) {
  PelletResult res;
  res.available = true;
  res.valid = true;
  res.distance_cm = distance_cm;

  const float fuellhoehe = cfg.tank_height_cm - distance_cm;
  const float span = cfg.tank_height_cm - cfg.min_distance_cm;
  float pct = 0.0f;
  if (span > 0.0001f) {
    pct = (fuellhoehe / span) * 100.0f;
  }
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  res.level_percent = pct;
  res.remaining_kg = (pct / 100.0f) * cfg.capacity_kg;
  res.low = pct < cfg.low_threshold_percent;
  return res;
}

// Moving average filter over the last 5 valid measurements with error handling.
class PelletFilter {
 public:
  static constexpr size_t WINDOW_SIZE = 5;
  static constexpr int MAX_CONSECUTIVE_ERRORS = 3;

  void recordSuccess(float distance_cm) {
    consecutive_errors_ = 0;
    if (history_.size() >= WINDOW_SIZE) {
      history_.pop_front();
    }
    history_.push_back(distance_cm);
  }

  // Returns true if consecutive errors reached the threshold (causing a reset).
  bool recordError() {
    consecutive_errors_++;
    if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
      clear();
      return true;
    }
    return false;
  }

  bool hasValue() const {
    return !history_.empty();
  }

  float averageDistance() const {
    if (history_.empty()) return 0.0f;
    float sum = 0.0f;
    for (float v : history_) sum += v;
    return sum / static_cast<float>(history_.size());
  }

  void clear() {
    history_.clear();
  }

  int consecutiveErrors() const {
    return consecutive_errors_;
  }

  size_t count() const {
    return history_.size();
  }

 private:
  std::deque<float> history_;
  int consecutive_errors_ = 0;
};

}  // namespace pellet
}  // namespace firenet
