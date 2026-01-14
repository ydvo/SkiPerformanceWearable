#pragma once

#include "i2c.hpp"
#include "icm20948.hpp"
#include "icm20948_detail.hpp"
#include "madgwick_filter.hpp"
#include <cstdint>

using ICM = espp::Icm20948<espp::icm20948::Interface::I2C>;

namespace SENSORS {
/* Constants */

/* Imu */
class Imu {
public:
  /* Vars */
  // I2C
  static constexpr uint8_t ICM20948_ADDRESS{0x69};
  static constexpr int ICM20948_I2C_HZ{400000};

  // Sensor Ranges
  static constexpr espp::icm20948::AccelerometerRange ACCELEROMETER_RANGE =
      espp::icm20948::AccelerometerRange::RANGE_2G;

  static constexpr espp::icm20948::GyroscopeRange GYROSCOPE_RANGE =
      espp::icm20948::GyroscopeRange::RANGE_250DPS;

  static constexpr espp::icm20948::MagnetometerMode MAGNETOMETER_MODE =
      espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_100_HZ;

  // Madgwick tuning value
  static constexpr float MADGWICK_BETA{0.1f}; // default filter beta

  /* Structs */
  struct Value {
    float roll{};
    float pitch{};
    float yaw{};
  };

  struct Raw {
    espp::icm20948::Value accel{};
    espp::icm20948::Value gyro{};
    espp::icm20948::Value mag{};
    float temperature{};
  };

  /* Methods */

  // Constructor
  explicit Imu(espp::I2c &i2c);

  explicit Imu(ICM::Config cfg);

  // initialize
  bool init();
  // whoami
  uint8_t get_whoami();

  // Update IMU with timestep dt (s), returns true if successful
  bool update(float dt);

  // Get filtered orientation (Madgwick)
  Value get_orientation() const {
    return orientation_;
  }

  // Get raw IMU data
  Raw get_raw() const {
    return raw_;
  }

private:
  ICM::Config make_default_config(espp::I2c &i2c);
  ICM imu_;             // icm20948 instance
  Value orientation_{}; // Filtered orientation data
  Raw raw_{};           // Raw orientation data
};
} // namespace SENSORS
