#pragma once

#include "i2c.hpp"
#include "icm20948.hpp"
#include "icm20948_detail.hpp"
#include "madgwick_filter_quat.hpp"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iterator>

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
  static constexpr float MADGWICK_BETA{0.5f}; // default filter beta

  // Low Pass Filter Bandwidth
  static constexpr espp::icm20948::AccelerometerFilterBandwidth ACCEL_LPF_BANDWIDTH{
      espp::icm20948::AccelerometerFilterBandwidth::BW_24_HZ};

  static constexpr espp::icm20948::GyroscopeFilterBandwidth GYRO_LPF_BANDWIDTH{
      espp::icm20948::GyroscopeFilterBandwidth::BW_11_HZ};

  /* Structs */
  struct Value {
    float x{};
    float y{};
    float z{};
  };

  struct Quaternion {
    float w{};
    float x{};
    float y{};
    float z{};
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
  bool update_raw(float dt);
  bool update(float dt);

  // Get filtered orientation (Madgwick)
  Quaternion get_orientation() const {
    return orientation_;
  }

  // Get raw IMU data
  Raw get_raw() const {
    return raw_;
  }

  /* Magnetometer */

  // disable magnetometer
  void disable_magnetometer() {
    enable_magnetometer_ = false;
  }

  //  enable magnetometer
  void enable_magnetometer() {
    enable_magnetometer_ = true;
  }

  // set hard iron
  void set_mag_hard_iron_bias(double b[3]) {
    memcpy(b_, b, sizeof(&b));
  }

  // set soft iron
  void set_mag_soft_iron_bias(double A[3][3]) {
    memcpy(A_, A, sizeof(&A));
  }

private:
  Value apply_mag_cal(espp::icm20948::Value raw); // helper function to apply magnetomer calibration

  ICM::Config make_default_config(espp::I2c &i2c);
  ICM imu_;                         // icm20948 instance
  espp::MadgwickFilterQuat filter_; // madgwick filter instance
  bool enable_magnetometer_;        // Controls if magnetometer is used
  Quaternion orientation_{};        // Filtered orientation data
  Raw raw_{};                       // Raw orientation data

  // initial biases. Values generated during initial 1 time calibration from py script
  double b_[3] = {3.795706, 15.694968, 25.832445}; // hard iron bias for magnetometer
  double A_[3][3] = {
      // soft iron correction matrix
      {1.68545815e01, 4.21150134e-03, 1.623333486e-01},
      {4.21150134e-03, 1.59097586e01, -2.21327219e-02},
      {1.62333486e-01, -2.21327219e-02, 1.64177326e01},
  };
};
} // namespace SENSORS
