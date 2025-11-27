#pragma once

#include "I2C.hpp"
#include "icm20948.hpp"
#include "icm20948_detail.hpp"
#include "madgwick_filter.hpp"
#include <cstdint>

namespace SENSORS {
/* Constants */

// I2C
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr int ICM20948_I2C_HZ{400000};

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
/* Imu */
class Imu {
public:
  // imu configuration parameters
  struct Config {
    uint8_t device_address;                                      // i2c address
    std::function<bool(uint8_t, const uint8_t *, size_t)> write; // i2c write function
    std::function<bool(uint8_t, uint8_t *, size_t)> read;        // i2c read function

    espp::icm20948::ImuConfig imu_config;

    float madgwick_beta = 0.1f; // default filter beta
    bool auto_init = true;
  };

  // Constructor
  explicit Imu(Common::I2C &i2c);
  explicit Imu(const Config &config);

  // whoami
  uint8_t get_whoami();

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
  Config make_default_config(Common::I2C &i2c);
  espp::Icm20948<espp::icm20948::Interface::I2C> imu_; // icm20948 instance
  espp::MadgwickFilter madgwick_;                      // sensor fusion algorithm (madgwick)
  ImuValue orientation_{};                             // Filtered orientation data
  ImuRaw raw_{};                                       // Raw orientation data
};
} // namespace SENSORS
