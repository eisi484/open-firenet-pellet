// Host tests for pellet calculation and filtering (open-firenet/pellet_calc.h).
#include "pellet_calc.h"
#include <iostream>
#include <cmath>

using namespace firenet::pellet;

static int ok = 0, ko = 0;
static void CH(const char* n, bool c) {
  if (c) ok++;
  else {
    ko++;
    std::cout << "ECHEC " << n << "\n";
  }
}

int main() {
  // --- 1. Config validation
  {
    PelletConfig cfg;
    CH("config: default is valid", cfg.isValid());
    CH("config: default enabled is false", !cfg.enabled);

    PelletConfig invalidMin = cfg;
    invalidMin.min_distance_cm = 75.0f;
    CH("config: min_distance >= tank_height is invalid", !invalidMin.isValid());

    PelletConfig invalidCap = cfg;
    invalidCap.capacity_kg = 0.0f;
    CH("config: capacity <= 0 is invalid", !invalidCap.isValid());

    PelletConfig invalidLow = cfg;
    invalidLow.low_threshold_percent = 105.0f;
    CH("config: low_threshold > 100 is invalid", !invalidLow.isValid());

    PelletConfig invalidPin = cfg;
    invalidPin.sda_pin = 19;  // USB D-
    CH("config: SDA on USB pin 19 is invalid", !invalidPin.isValid());
    invalidPin.sda_pin = 20;  // USB D+
    CH("config: SDA on USB pin 20 is invalid", !invalidPin.isValid());
    invalidPin.sda_pin = 0;   // Boot pin
    CH("config: SDA on Boot pin 0 is invalid", !invalidPin.isValid());

    PelletConfig samePin = cfg;
    samePin.sda_pin = 9;
    samePin.scl_pin = 9;
    CH("config: SDA == SCL is invalid", !samePin.isValid());
  }

  // --- 2. Reference calculation: 49.2 cm -> 29.9% -> 13 kg
  {
    PelletConfig cfg;
    cfg.tank_height_cm = 70.0f;
    cfg.min_distance_cm = 0.5f;
    cfg.capacity_kg = 45.0f;
    cfg.low_threshold_percent = 20.0f;

    PelletResult r1 = calculatePelletLevel(49.2f, cfg);
    // fuellhoehe = 70 - 49.2 = 20.8
    // span = 70 - 0.5 = 69.5
    // pct = 20.8 / 69.5 * 100 = 29.928%
    CH("reference 49.2cm: level_percent ~ 29.9%", std::abs(r1.level_percent - 29.9f) <= 0.1f);
    CH("reference 49.2cm: remaining_kg ~ 13 kg", std::round(r1.remaining_kg) == 13.0f);
    CH("reference 49.2cm: low flag is false", !r1.low);

    // 49.3 cm -> 29.8%
    PelletResult r2 = calculatePelletLevel(49.3f, cfg);
    CH("reference 49.3cm: level_percent ~ 29.8%", std::abs(r2.level_percent - 29.8f) <= 0.1f);
  }

  // --- 2b. Live calibrated case: 40 kg tank, 65.0 cm height, 10.3 cm min_dist
  {
    PelletConfig cfg; // uses defaults: 65.0 cm, 10.3 cm, 40.0 kg
    PelletResult empty = calculatePelletLevel(65.0f, cfg);
    CH("live: 65.0 cm is 0.0%", std::abs(empty.level_percent - 0.0f) < 0.1f);
    CH("live: 65.0 cm is 0.0 kg", std::abs(empty.remaining_kg - 0.0f) < 0.1f);

    PelletResult refill30 = calculatePelletLevel(24.1f, cfg);
    CH("live: 24.1 cm is ~74.8%", std::abs(refill30.level_percent - 74.8f) < 0.5f);
    CH("live: 24.1 cm is ~29.9 kg", std::abs(refill30.remaining_kg - 29.9f) < 0.5f);

    PelletResult full = calculatePelletLevel(10.3f, cfg);
    CH("live: 10.3 cm is 100.0%", std::abs(full.level_percent - 100.0f) < 0.1f);
    CH("live: 10.3 cm is 40.0 kg", std::abs(full.remaining_kg - 40.0f) < 0.1f);
  }

  // --- 3. Clamping and low threshold
  {
    PelletConfig cfg;
    cfg.tank_height_cm = 70.0f;
    cfg.min_distance_cm = 0.5f;
    cfg.capacity_kg = 45.0f;
    cfg.low_threshold_percent = 20.0f;

    // Full tank: distance < min_distance
    PelletResult full = calculatePelletLevel(0.0f, cfg);
    CH("clamping: full tank clamped to 100%", full.level_percent == 100.0f);
    CH("clamping: full tank capacity 45kg", full.remaining_kg == 45.0f);
    CH("clamping: full tank not low", !full.low);

    // Completely empty tank: distance > tank_height
    PelletResult empty = calculatePelletLevel(80.0f, cfg);
    CH("clamping: empty tank clamped to 0%", empty.level_percent == 0.0f);
    CH("clamping: empty tank remaining 0kg", empty.remaining_kg == 0.0f);
    CH("clamping: empty tank is low", empty.low);

    // Exactly at threshold boundary
    PelletResult lowWarn = calculatePelletLevel(60.0f, cfg);
    // fuellhoehe = 10, span = 69.5 -> pct = 14.38% < 20%
    CH("low threshold: 14.4% is low", lowWarn.low);
  }

  // --- 4. Moving average filter & outlier / error handling (3 errors reset)
  {
    PelletFilter filter;
    CH("filter: starts empty", !filter.hasValue());

    filter.recordSuccess(50.0f);
    CH("filter: 1 value", filter.hasValue() && filter.count() == 1);
    CH("filter: avg 50.0", filter.averageDistance() == 50.0f);

    filter.recordSuccess(52.0f);
    filter.recordSuccess(48.0f);
    filter.recordSuccess(50.0f);
    filter.recordSuccess(50.0f);
    CH("filter: 5 values", filter.count() == 5);
    CH("filter: avg of [50, 52, 48, 50, 50] == 50.0", std::abs(filter.averageDistance() - 50.0f) < 0.001f);

    // 6th value pushes out oldest
    filter.recordSuccess(60.0f);
    CH("filter: max 5 values kept", filter.count() == 5);
    // [52, 48, 50, 50, 60] sum = 260 / 5 = 52.0
    CH("filter: window shifted, avg 52.0", std::abs(filter.averageDistance() - 52.0f) < 0.001f);

    // 1st error: filter retains history
    bool reset1 = filter.recordError();
    CH("filter: 1st error does not reset", !reset1 && filter.hasValue() && filter.consecutiveErrors() == 1);

    // 2nd error: filter still retains history
    bool reset2 = filter.recordError();
    CH("filter: 2nd error does not reset", !reset2 && filter.hasValue() && filter.consecutiveErrors() == 2);

    // 3rd consecutive error: triggers reset, clears history
    bool reset3 = filter.recordError();
    CH("filter: 3rd error triggers reset", reset3 && !filter.hasValue() && filter.count() == 0);

    // Recovery after failure: new valid measurement restores value
    filter.recordSuccess(45.0f);
    CH("filter: recovered after failure", filter.hasValue() && filter.count() == 1);
    CH("filter: new average is 45.0", filter.averageDistance() == 45.0f);
    CH("filter: consecutive errors reset to 0", filter.consecutiveErrors() == 0);
  }

  std::cout << "pellet calc test: " << ok << " ok, " << ko << " ko\n";
  return ko ? 1 : 0;
}
