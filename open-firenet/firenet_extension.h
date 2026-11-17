#pragma once
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include "firenet_i2c_bus.h"

namespace firenet {

struct ConfigField {
  enum Type { INT, FLOAT, BOOL, STRING };
  const char* key;
  const char* label;
  Type type;
  const char* default_val;
  const char* unit;
  float min_val;
  float max_val;
};

struct ExtensionValue {
  enum Kind {
    GENERIC,
    DISTANCE,
    PERCENT,
    MASS,
    TEMPERATURE,
    HUMIDITY,
    BINARY_ALERT
  };

  const char* key;
  const char* label;
  Kind kind;
  const char* unit;
  bool is_binary;
  float num_val;
  bool bool_val;
  bool valid;
};

class Extension {
 public:
  virtual ~Extension() = default;

  // Unique identifier for this extension, e.g. "pellet_level"
  virtual const char* id() const = 0;

  // Human-readable display name, e.g. "Pellet Level Sensor"
  virtual const char* name() const = 0;

  // Optional I2C 7-bit slave address, or 0 if not an I2C device
  virtual uint8_t i2cAddress() const { return 0; }

  // Configuration schema for this extension
  virtual size_t configFieldCount() const = 0;
  virtual const ConfigField* configFields() const = 0;

  // Lifecycle
  virtual bool begin(const std::map<std::string, std::string>& config, I2CBus& bus, std::string* error_msg) = 0;
  virtual void stop() = 0;
  virtual void poll() = 0;

  // Status & Health
  virtual bool isRunning() const = 0;
  virtual bool isHealthy() const = 0;
  virtual const char* statusMessage() const = 0;

  // Output telemetry
  virtual size_t valueCount() const = 0;
  virtual const ExtensionValue* values() const = 0;
};

} // namespace firenet
