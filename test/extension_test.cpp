#include <cassert>
#include <iostream>
#include "firenet_pin_registry.h"
#include "firenet_i2c_bus.h"
#include "firenet_extension.h"
#include "firenet_extension_manager.h"
#include "pellet_extension.h"

int main() {
  std::cout << "Running extension & pin registry tests..." << std::endl;

  // 1. Test PinRegistry
  {
    firenet::PinRegistry reg;
    std::string err;

    // Hardwired pins must be rejected
    assert(!reg.isAvailable(0, &err)); // Boot button
    assert(!reg.isAvailable(19, &err)); // USB CDC D-
    assert(!reg.isAvailable(20, &err)); // USB CDC D+
    assert(!reg.isAvailable(26, &err)); // SPI Flash
    assert(!reg.isAvailable(43, &err)); // UART0 TX

    // Valid free pins
    assert(reg.isAvailable(2, &err));
    assert(reg.isAvailable(4, &err));

    // Reserve pins
    assert(reg.reserve(2, "i2c_sda", &err));
    assert(reg.reserve(4, "i2c_scl", &err));

    // Re-reserving already in use pin must fail
    assert(!reg.reserve(2, "other_device", &err));
    assert(err.find("already in use by i2c_sda") != std::string::npos);

    // Releasing pin makes it available again
    reg.release(2);
    assert(reg.isAvailable(2, &err));
    assert(reg.reserve(2, "new_owner", &err));

    // Board display reservations
    firenet::PinRegistry reg_dongle;
    reg_dongle.reserveBoardDisplay("pocket_dongle");
    assert(!reg_dongle.isAvailable(10, &err));
    assert(!reg_dongle.isAvailable(14, &err));
    assert(reg_dongle.isAvailable(2, &err));

    firenet::PinRegistry reg_tdongle;
    reg_tdongle.reserveBoardDisplay("tdongle");
    assert(!reg_tdongle.isAvailable(5, &err));
    assert(!reg_tdongle.isAvailable(38, &err));
  }
  std::cout << "PinRegistry checks: OK" << std::endl;

  // 2. Test I2CBus
  {
    firenet::I2CBus bus;
    assert(!bus.isReady());
    assert(bus.begin(2, 4, 100000));
    assert(bus.isReady());
    assert(bus.sda() == 2);
    assert(bus.scl() == 4);

    bus.setMockProbeResult(true);
    assert(bus.probe(0x29));

    bus.setMockProbeResult(false);
    assert(!bus.probe(0x29));

    bus.end();
    assert(!bus.isReady());
  }
  std::cout << "I2CBus checks: OK" << std::endl;

  // 3. Test PelletExtension
  {
    firenet::PelletExtension pellet;
    assert(std::string(pellet.id()) == "pellet_level");
    assert(pellet.i2cAddress() == 0x29);
    assert(pellet.configFieldCount() == 4);

    firenet::I2CBus bus;
    bus.begin(2, 4);
    bus.setMockProbeResult(true);

    std::map<std::string, std::string> cfg = {
      {"tank_height_cm", "65.0"},
      {"min_distance_cm", "10.3"},
      {"capacity_kg", "40.0"},
      {"low_threshold_percent", "20.0"}
    };

    std::string err;
    assert(pellet.begin(cfg, bus, &err));
    assert(pellet.isRunning());
    assert(pellet.isHealthy());

    pellet.poll();
    assert(pellet.valueCount() == 4);

    const auto* vals = pellet.values();
    assert(std::string(vals[0].key) == "distance_cm");
    assert(vals[0].valid);
    assert(std::string(vals[1].key) == "level_percent");
    assert(vals[1].valid);
    assert(std::string(vals[2].key) == "remaining_kg");
    assert(vals[2].valid);
    assert(std::string(vals[3].key) == "low");
    assert(vals[3].valid);

    pellet.stop();
    assert(!pellet.isRunning());
  }
  std::cout << "PelletExtension checks: OK" << std::endl;

  // 4. Test ExtensionManager
  {
    firenet::ExtensionManager mgr;
    firenet::PelletExtension pellet;

    mgr.registerExtension(&pellet);
    assert(mgr.findExtension("pellet_level") == &pellet);
    assert(mgr.findExtension("nonexistent") == nullptr);

    // Test configure I2C pins with pin collision detection
    std::string err;
    assert(!mgr.configureI2CPins(19, 20, &err)); // USB CDC pins -> rejected
    assert(err.find("reserved by system") != std::string::npos);

    assert(mgr.configureI2CPins(2, 4, &err)); // Valid pins
    assert(mgr.i2cBus().sda() == 2);
    assert(mgr.i2cBus().scl() == 4);

    // Enable extension
    std::map<std::string, std::string> cfg = {
      {"tank_height_cm", "60.0"},
      {"capacity_kg", "35.0"}
    };
    mgr.i2cBus().setMockProbeResult(true);
    assert(mgr.setExtensionConfig("pellet_level", true, cfg, &err));
    assert(mgr.isEnabled("pellet_level"));
    assert(pellet.isRunning());

    // Generate JSON state for /api/state
    std::string state_json = mgr.jsonState();
    assert(state_json.find("\"extensions\":{\"pellet_level\":{\"enabled\":true") != std::string::npos);
    assert(state_json.find("\"remaining_kg\":") != std::string::npos);
    assert(state_json.find("\"distance_cm\":") != std::string::npos);

    // Generate JSON for /api/extensions
    std::string ext_json = mgr.jsonExtensions();
    assert(ext_json.find("\"i2c\":{\"ready\":true,\"sda\":2,\"scl\":4}") != std::string::npos);
    assert(ext_json.find("\"id\":\"pellet_level\"") != std::string::npos);
    assert(ext_json.find("\"config_fields\":[") != std::string::npos);

    // Disable extension
    assert(mgr.setExtensionConfig("pellet_level", false, cfg, &err));
    assert(!mgr.isEnabled("pellet_level"));
    assert(!pellet.isRunning());
  }
  std::cout << "ExtensionManager checks: OK" << std::endl;

  std::cout << "\nAll extension system tests PASSED!" << std::endl;
  return 0;
}
