#pragma once

#include "icm20948.hpp"
#include "icm20948_detail.hpp"
#include "madgwick_filter.hpp"

/* Constants */
// Sensor Ranges
static constexpr espp::icm20948::AccelerometerRange ACCELEROMETER_RANGE =
    espp::icm20948::AccelerometerRange::RANGE_2G;

static constexpr espp::icm20948::GyroscopeRange GYROSCOPE_RANGE =
    espp::icm20948::GyroscopeRange::RANGE_250DPS;

static constexpr espp::icm20948::MagnetometerMode MAGNETOMETER_MODE =
    espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_100_HZ;

struct ImuValue {
  float roll{};
  float pitch{};
  float yaw{};
};

struct ImuRaw {
  espp::icm20948::Value accel{};
  espp::icm20948::Value gyro{};
  espp::icm20948::Value mag{};
  float temperature{};
};

class Imu {
public:
  struct Config {
    uint8_t device_address;
    std::function<bool(uint8_t, const uint8_t *, size_t)> write;
    std::function<bool(uint8_t, uint8_t *, size_t)> read;

    espp::icm20948::ImuConfig imu_config;
    float madgwick_beta = 0.1f; // default filter beta
    bool auto_init = true;
  };

  explicit Imu(const Config &config);

  // Update IMU with timestep dt (s), returns true if successful
  bool update(float dt);

  // Get filtered orientation (Madgwick)
  ImuValue get_orientation() const {
    return orientation_;
  }

  // Get raw IMU data
  ImuRaw get_raw() const {
    return raw_;
  }

private:
  espp::Icm20948<espp::icm20948::Interface::I2C> imu_;
  espp::MadgwickFilter madgwick_;
  ImuValue orientation_{};
  ImuRaw raw_{};
};
