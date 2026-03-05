/**
 * @file imu.hpp
 * @ingroup sensors
 * @brief IMU wrapper using ICM‑20948 and Madgwick filter.
 *
 * Provides a high‑level interface to the ICM‑20948 9‑axis sensor. Handles
 * initialization, raw data acquisition, magnetometer calibration, and orientation
 * estimation via a Madgwick quaternion filter.
 */
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
/**
 * @brief High‑level IMU driver.
 *
 * Wraps an `espp::Icm20948` instance and a `MadgwickFilterQuat` to provide
 * calibrated raw sensor data and filtered orientation (quaternion and Euler
 * angles). Supports optional magnetometer usage with hard‑iron and soft‑iron
 * calibration.
 *
 * Thread‑safety: methods are not internally synchronized; callers should ensure
 * exclusive access when used from multiple FreeRTOS tasks.
 */
class Imu {
public:
  /* Vars */
  // I2C
  static constexpr uint8_t ICM20948_ADDRESS{0x69};
  static constexpr int ICM20948_I2C_HZ{400000};

  // delay after startup before configuration can start
  static constexpr uint8_t INIT_DELAY{100};

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
  /**
   * @brief 3‑axis vector of float values.
+   *
+   * Used for accelerometer, gyro, and magnetometer data after scaling to
+   * physical units.
+   */
  struct Value {
    float x{}; ///< X component.
    float y{}; ///< Y component.
    float z{}; ///< Z component.
  };

  /**
   * @brief Quaternion representing orientation.
+   *
+   * The filter outputs a unit quaternion (w, x, y, z).
+   */
  struct Quaternion {
    float w{}; ///< Scalar component.
    float x{}; ///< X vector component.
    float y{}; ///< Y vector component.
    float z{}; ///< Z vector component.
  };

  /**
   * @brief Euler angles (degrees) derived from quaternion.
+   *
+   * Pitch rotates about X, roll about Y, yaw about Z.
+   */
  struct Euler {
    float pitch{}; ///< Rotation about X axis (degrees).
    float roll{};  ///< Rotation about Y axis (degrees).
    float yaw{};   ///< Rotation about Z axis (degrees).
  };

  /**
   * @brief Raw sensor readings before any calibration.
+   *
+   * Accelerometer, gyroscope, and magnetometer values are in sensor units; the
+   * temperature is in degrees Celsius.
+   */
  struct Raw {
    espp::icm20948::Value accel{}; ///< Accelerometer raw data.
    espp::icm20948::Value gyro{};  ///< Gyroscope raw data.
    espp::icm20948::Value mag{};   ///< Magnetometer raw data.
    float temperature{};           ///< Temperature in °C.
  };

  /* Methods */

  // Constructor
  /**
   * @brief Construct an Imu using an I2C bus.
+   *
+   * @param i2c Reference to an initialized `espp::I2c` instance.
+   */
  explicit Imu(espp::I2c &i2c);

  /**
   * @brief Construct an Imu with a pre‑configured ICM‑20948 configuration.
+   *
+   * @param cfg Configuration struct for the underlying `espp::Icm20948` driver.
+   */
  explicit Imu(ICM::Config cfg);

  // initialize
  /**
   * @brief Initialize the IMU hardware.
+   *
+   * Performs sensor reset, configures low‑pass filters, and enables the device.
+   * Must be called before any data acquisition.
+   * @return true on success, false on error.
+   */
  bool init();

  // whoami
  /**
   * @brief Read the WHO_AM_I register.
+   *
+   * @return Device identifier byte.
+   */
  uint8_t get_whoami();

  // Update IMU with timestep dt (s), returns true if successful
  /**
   * @brief Update raw sensor readings.
+   *
+   * Reads accelerometer, gyroscope, magnetometer and temperature into the
+   * internal `raw_` structure.
+   * @param dt Time step in seconds.
+   * @return true on successful read, false otherwise.
+   */
  bool update_raw(float dt);
  bool update(float dt);

  // Get filtered orientation (Madgwick)
  Quaternion get_orientation() const {
    return orientation_;
  }

  // Get euler angles (degrees) from the Madgwick filter
  Euler get_euler() const;

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
    memcpy(b_, b, sizeof(b_));
  }

  // set soft iron
  void set_mag_soft_iron_bias(double A[3][3]) {
    memcpy(A_, A, sizeof(A_));
  }

  /** @brief Put the ICM-20948 into hardware sleep mode (~8 uA). */
  esp_err_t sleep();

  /** @brief Wake the ICM-20948 from hardware sleep mode. */
  esp_err_t wake();

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
