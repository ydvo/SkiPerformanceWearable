/*
 * fuelgauge.hpp
 *  - simple drive for MAX1704X L-ion fuel gauge adapted from Adafruit Arduino cpp library
 */

#pragma once
#include "i2c.hpp"
#include <stdint.h>

namespace POWER {

class fuelgauge {
public:
  enum REGISTERS {
    VCELL = 0x02,   // Cell voltage
    SOC = 0x04,     // cell state of charge
    MODE = 0x06,    // Manages mode
    VERSION = 0x08, // IC version
    HIBRT = 0x0A,   // manages hibernation
    CONFIG = 0x0C,  // manages configuration
    VALERT = 0x14,  // holds voltage alert values
    CRATE = 0x16,   // cell charge rate
    VRESET = 0x18,  // reset voltage setting
    CHIPID = 0x19,  // semi unique chip id
    STATUS = 0x1A,  // current alert/status
    CMD = 0xFE      // Written for special commands
  };

  static constexpr auto DEFAULT_ADDRESS{0x36};
  static constexpr auto VOLTAGE_SCALING_FACTOR{78.125e-6f};

  fuelgauge(espp::I2c &i2c);

  bool isDeviceReady();
  uint16_t getICversion();
  uint8_t getChipID();

  float cellVoltage(void);
  float cellPercent(void);
  float chargeRate(void);

private:
  espp::I2c &i2c_;
};
} // namespace POWER
