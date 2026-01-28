/*
 * fuelgauge.cpp
 *  - simple drive for MAX1704X L-ion fuel gauge adapted from Adafruit Arduino cpp library
 */

#include "fuelgauge.hpp"

using namespace POWER;

fuelgauge::fuelgauge(espp::I2c &i2c) : i2c_(i2c) {}

/*
 * Check if device is ready to be read from.
 * Chip ID = 0xFF and Version = 0xFFF if no battery is attached
 */
bool fuelgauge::isDeviceReady() {
  return (getICversion() & 0xFFF0) == 0x0010;
}

/*
 * Get IC LSI version
 */
uint16_t fuelgauge::getICversion() {
  uint8_t buffer[2] = {0};

  bool status =
      i2c_.read_at_register(DEFAULT_ADDRESS, fuelgauge::REGISTERS::VERSION, buffer, sizeof(buffer));

  if (status) {
    return uint16_t(buffer[0] << 8) | buffer[1];
  }

  return 0;
}

/*
 * Get semiunique chip id
 */
uint8_t fuelgauge::getChipID() {
  uint8_t buffer = 0;

  bool status =
      i2c_.read_at_register(DEFAULT_ADDRESS, fuelgauge::REGISTERS::CHIPID, &buffer, sizeof(buffer));

  if (status) {
    return buffer;
  }

  return 0;
}

/*
 *  Get battery voltage
 */
float fuelgauge::cellVoltage(void) {
  if (!isDeviceReady())
    return 0.0f;

  uint8_t buffer[2] = {0};

  bool status =
      i2c_.read_at_register(DEFAULT_ADDRESS, fuelgauge::REGISTERS::VCELL, buffer, sizeof(buffer));

  if (status) {
    float voltage = static_cast<float>(uint16_t(buffer[0] << 8) | buffer[1]);
    return voltage * VOLTAGE_SCALING_FACTOR;
  }

  return 0.0f;
}

/*
 * Get battery percentage
 */
float fuelgauge::cellPercent(void) {
  if (!isDeviceReady())
    return 0.0f;

  uint8_t buffer[2] = {0};

  bool status =
      i2c_.read_at_register(DEFAULT_ADDRESS, fuelgauge::REGISTERS::SOC, buffer, sizeof(buffer));

  if (status) {
    float percent = static_cast<float>(uint16_t(buffer[0] << 8) | buffer[1]);
    return percent / 256.0;
  }

  return 0.0f;
}

/*
 * Get charge or discharge rate in percent/hour
 */
float fuelgauge::chargeRate(void) {
  if (!isDeviceReady())
    return 0.0f;

  uint8_t buffer[2] = {0};

  bool status =
      i2c_.read_at_register(DEFAULT_ADDRESS, fuelgauge::REGISTERS::CRATE, buffer, sizeof(buffer));

  if (status) {
    float percent = static_cast<float>(static_cast<int16_t>(
        (static_cast<uint16_t>(buffer[0]) << 8) | static_cast<uint16_t>(buffer[1])));
    return percent * 0.208;
  }

  return 0.0f;
}
