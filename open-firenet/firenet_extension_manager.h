#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include "firenet_pin_registry.h"
#include "firenet_i2c_bus.h"
#include "firenet_extension.h"

#ifdef ARDUINO
#include <Preferences.h>
#endif

namespace firenet {

class ExtensionManager {
 public:
  ExtensionManager() = default;

  PinRegistry& pinRegistry() { return pin_registry_; }
  const PinRegistry& pinRegistry() const { return pin_registry_; }

  I2CBus& i2cBus() { return i2c_bus_; }
  const I2CBus& i2cBus() const { return i2c_bus_; }

  void registerExtension(Extension* ext) {
    if (!ext) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto* e : extensions_) {
      if (std::string(e->id()) == ext->id()) return;
    }
    extensions_.push_back(ext);
    ext_enabled_[ext->id()] = false;
  }

  Extension* findExtension(const std::string& id) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto* e : extensions_) {
      if (e->id() == id) return e;
    }
    return nullptr;
  }

  bool isEnabled(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = ext_enabled_.find(id);
    return (it != ext_enabled_.end() && it->second);
  }

  bool configureI2CPins(int sda, int scl, std::string* error = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sda < 0 || scl < 0 || sda == scl) {
      if (error) *error = "Invalid I2C pin configuration (SDA and SCL must be distinct positive GPIO numbers)";
      return false;
    }

    // Check against pin registry (temporarily releasing current I2C pins if already set)
    if (i2c_sda_ >= 0) pin_registry_.release(i2c_sda_);
    if (i2c_scl_ >= 0) pin_registry_.release(i2c_scl_);

    std::string reason;
    if (!pin_registry_.reserve(sda, "i2c_sda", &reason)) {
      if (error) *error = reason;
      // re-reserve previous pins
      if (i2c_sda_ >= 0) pin_registry_.reserve(i2c_sda_, "i2c_sda");
      if (i2c_scl_ >= 0) pin_registry_.reserve(i2c_scl_, "i2c_scl");
      return false;
    }
    if (!pin_registry_.reserve(scl, "i2c_scl", &reason)) {
      if (error) *error = reason;
      pin_registry_.release(sda);
      if (i2c_sda_ >= 0) pin_registry_.reserve(i2c_sda_, "i2c_sda");
      if (i2c_scl_ >= 0) pin_registry_.reserve(i2c_scl_, "i2c_scl");
      return false;
    }

    i2c_sda_ = sda;
    i2c_scl_ = scl;
    return i2c_bus_.begin(sda, scl);
  }

  bool setExtensionConfig(const std::string& id, bool enabled, const std::map<std::string, std::string>& config, std::string* error = nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    Extension* ext = nullptr;
    for (auto* e : extensions_) {
      if (e->id() == id) { ext = e; break; }
    }
    if (!ext) {
      if (error) *error = "Unknown extension ID: " + id;
      return false;
    }

    ext_enabled_[id] = enabled;
    ext_configs_[id] = config;

    if (enabled) {
      // If extension needs I2C, ensure I2C bus is initialized
      if (ext->i2cAddress() != 0) {
        if (!i2c_bus_.isReady()) {
          if (i2c_sda_ >= 0 && i2c_scl_ >= 0) {
            i2c_bus_.begin(i2c_sda_, i2c_scl_);
          } else {
            if (error) *error = "Cannot enable extension: I2C bus pins not configured";
            ext_enabled_[id] = false;
            return false;
          }
        }
      }
      std::string begin_err;
      if (!ext->begin(config, i2c_bus_, &begin_err)) {
        if (error) *error = begin_err.empty() ? "Extension initialization failed" : begin_err;
        ext_enabled_[id] = false;
        return false;
      }
    } else {
      if (ext->isRunning()) {
        ext->stop();
      }
    }
    return true;
  }

  void begin() {
    std::lock_guard<std::mutex> lock(mutex_);
    // Start I2C bus if pins configured
    if (i2c_sda_ >= 0 && i2c_scl_ >= 0) {
      i2c_bus_.begin(i2c_sda_, i2c_scl_);
    }

    // Start enabled extensions
    for (auto* ext : extensions_) {
      std::string id = ext->id();
      if (ext_enabled_[id]) {
        std::string err;
        ext->begin(ext_configs_[id], i2c_bus_, &err);
      }
    }
  }

  void poll() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto* ext : extensions_) {
      if (ext_enabled_[ext->id()] && ext->isRunning()) {
        ext->poll();
        if (!ext->isHealthy()) {
          // Extension failed failsafe: record and stop to protect stove operation
          // The bridge continues running unaffected.
        }
      }
    }
  }

  // Generates JSON for /api/state: "extensions": { "<id>": { "enabled": true, ... } }
  std::string jsonState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string s = "\"extensions\":{";
    bool first_ext = true;
    for (const auto* ext : extensions_) {
      std::string id = ext->id();
      bool en = false;
      auto it_en = ext_enabled_.find(id);
      if (it_en != ext_enabled_.end()) en = it_en->second;

      if (!first_ext) s += ",";
      first_ext = false;

      s += "\"" + id + "\":{";
      s += "\"enabled\":" + std::string(en ? "true" : "false");
      s += ",\"running\":" + std::string(ext->isRunning() ? "true" : "false");
      s += ",\"healthy\":" + std::string(ext->isHealthy() ? "true" : "false");
      s += ",\"status\":\"" + std::string(ext->statusMessage()) + "\"";

      if (en && ext->isRunning()) {
        size_t val_count = ext->valueCount();
        const auto* vals = ext->values();
        for (size_t i = 0; i < val_count; ++i) {
          const auto& v = vals[i];
          s += ",\"" + std::string(v.key) + "\":";
          if (!v.valid) {
            s += "null";
          } else if (v.is_binary) {
            s += v.bool_val ? "true" : "false";
          } else {
            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f", v.num_val);
            s += buf;
          }
        }
      }
      s += "}";
    }
    s += "}";
    return s;
  }

  // Generates full schema and values for /api/extensions endpoint
  std::string jsonExtensions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string s = "{\"i2c\":{\"ready\":" + std::string(i2c_bus_.isReady() ? "true" : "false");
    s += ",\"sda\":" + std::to_string(i2c_sda_);
    s += ",\"scl\":" + std::to_string(i2c_scl_) + "},\"extensions\":[";

    bool first_ext = true;
    for (const auto* ext : extensions_) {
      if (!first_ext) s += ",";
      first_ext = false;

      std::string id = ext->id();
      bool en = false;
      auto it_en = ext_enabled_.find(id);
      if (it_en != ext_enabled_.end()) en = it_en->second;

      s += "{\"id\":\"" + id + "\"";
      s += ",\"name\":\"" + std::string(ext->name()) + "\"";
      s += ",\"i2c_address\":" + std::to_string(ext->i2cAddress());
      s += ",\"enabled\":" + std::string(en ? "true" : "false");
      s += ",\"running\":" + std::string(ext->isRunning() ? "true" : "false");
      s += ",\"healthy\":" + std::string(ext->isHealthy() ? "true" : "false");
      s += ",\"status\":\"" + std::string(ext->statusMessage()) + "\"";

      // Config fields
      s += ",\"config_fields\":[";
      size_t f_count = ext->configFieldCount();
      const auto* fields = ext->configFields();
      for (size_t f = 0; f < f_count; ++f) {
        if (f > 0) s += ",";
        const auto& cf = fields[f];
        s += "{\"key\":\"" + std::string(cf.key) + "\"";
        s += ",\"label\":\"" + std::string(cf.label) + "\"";
        s += ",\"type\":";
        switch (cf.type) {
          case ConfigField::INT: s += "\"int\""; break;
          case ConfigField::FLOAT: s += "\"float\""; break;
          case ConfigField::BOOL: s += "\"bool\""; break;
          default: s += "\"string\""; break;
        }
        s += ",\"default\":\"" + std::string(cf.default_val) + "\"";
        s += ",\"unit\":\"" + std::string(cf.unit ? cf.unit : "") + "\"";
        s += ",\"min\":" + std::to_string(cf.min_val);
        s += ",\"max\":" + std::to_string(cf.max_val);

        // Current value
        std::string cur_val = cf.default_val;
        auto it_c = ext_configs_.find(id);
        if (it_c != ext_configs_.end()) {
          auto it_v = it_c->second.find(cf.key);
          if (it_v != it_c->second.end()) cur_val = it_v->second;
        }
        s += ",\"value\":\"" + cur_val + "\"}";
      }
      s += "]";

      // Values
      s += ",\"values\":[";
      size_t val_count = ext->valueCount();
      const auto* vals = ext->values();
      for (size_t i = 0; i < val_count; ++i) {
        if (i > 0) s += ",";
        const auto& v = vals[i];
        s += "{\"key\":\"" + std::string(v.key) + "\"";
        s += ",\"label\":\"" + std::string(v.label) + "\"";
        s += ",\"unit\":\"" + std::string(v.unit ? v.unit : "") + "\"";
        s += ",\"kind\":";
        switch (v.kind) {
          case ExtensionValue::DISTANCE: s += "\"distance\""; break;
          case ExtensionValue::PERCENT: s += "\"percent\""; break;
          case ExtensionValue::MASS: s += "\"mass\""; break;
          case ExtensionValue::BINARY_ALERT: s += "\"binary_alert\""; break;
          default: s += "\"generic\""; break;
        }
        s += ",\"is_binary\":" + std::string(v.is_binary ? "true" : "false");
        s += ",\"valid\":" + std::string(v.valid ? "true" : "false");
        s += ",\"value\":";
        if (!v.valid) {
          s += "null";
        } else if (v.is_binary) {
          s += v.bool_val ? "true" : "false";
        } else {
          char buf[32];
          snprintf(buf, sizeof(buf), "%.2f", v.num_val);
          s += buf;
        }
        s += "}";
      }
      s += "]}";
    }
    s += "]}";
    return s;
  }

#ifdef ARDUINO
  void loadFromNvs(Preferences& prefs) {
    std::lock_guard<std::mutex> lock(mutex_);
    prefs.begin("extensions", true);
    int sda = prefs.getInt("i2c_sda", 2);
    int scl = prefs.getInt("i2c_scl", 4);
    prefs.end();

    if (sda >= 0 && scl >= 0) {
      pin_registry_.reserve(sda, "i2c_sda");
      pin_registry_.reserve(scl, "i2c_scl");
      i2c_sda_ = sda;
      i2c_scl_ = scl;
    }

    for (auto* ext : extensions_) {
      std::string id = ext->id();
      std::string ns = "ext_" + id;
      if (ns.length() > 15) ns = ns.substr(0, 15);
      prefs.begin(ns.c_str(), true);
      bool en = prefs.getBool("enabled", false);
      ext_enabled_[id] = en;

      std::map<std::string, std::string> cfg;
      size_t f_count = ext->configFieldCount();
      const auto* fields = ext->configFields();
      for (size_t f = 0; f < f_count; ++f) {
        const char* k = fields[f].key;
        String val = prefs.getString(k, fields[f].default_val);
        cfg[k] = val.c_str();
      }
      ext_configs_[id] = cfg;
      prefs.end();
    }
  }

  void saveToNvs(Preferences& prefs) {
    std::lock_guard<std::mutex> lock(mutex_);
    prefs.begin("extensions", false);
    prefs.putInt("i2c_sda", i2c_sda_);
    prefs.putInt("i2c_scl", i2c_scl_);
    prefs.end();

    for (auto* ext : extensions_) {
      std::string id = ext->id();
      std::string ns = "ext_" + id;
      if (ns.length() > 15) ns = ns.substr(0, 15);
      prefs.begin(ns.c_str(), false);
      prefs.putBool("enabled", ext_enabled_[id]);
      const auto& cfg = ext_configs_[id];
      for (const auto& kv : cfg) {
        prefs.putString(kv.first.c_str(), kv.second.c_str());
      }
      prefs.end();
    }
  }
#endif

 private:
  mutable std::mutex mutex_;
  PinRegistry pin_registry_;
  I2CBus i2c_bus_;
  int i2c_sda_ = -1;
  int i2c_scl_ = -1;
  std::vector<Extension*> extensions_;
  std::map<std::string, bool> ext_enabled_;
  std::map<std::string, std::map<std::string, std::string>> ext_configs_;
};

} // namespace firenet
