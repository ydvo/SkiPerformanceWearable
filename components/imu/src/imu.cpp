#include "imu.hpp"
#include "fast_math.hpp"
#include <system_error>

namespace SENSORS {

// Forward declaration
Imu::Value calibrate_mag(espp::icm20948::Value raw);

// constructor
Imu::Imu(espp::I2c &i2c) : Imu(make_default_config(i2c)) {}

Imu::Imu(ICM::Config cfg) : imu_(cfg), filter_(MADGWICK_BETA) {}

// Construct default config from i2c
ICM::Config Imu::make_default_config(espp::I2c &i2c) {
  ICM::Config cfg{.device_address = ICM20948_ADDRESS,
                  .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                     std::placeholders::_2, std::placeholders::_3),
                  .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3),
                  .imu_config = {.accelerometer_range = ACCELEROMETER_RANGE,
                                 .gyroscope_range = GYROSCOPE_RANGE,
                                 .accelerometer_sample_rate_divider = 9, // 1kHz / (1 + 9) = 100Hz
                                 .gyroscope_sample_rate_divider = 9,     // 1kHz / (1 + 9) = 100Hz
                                 .magnetometer_mode = MAGNETOMETER_MODE},
                  .auto_init = false};
  return cfg;
}

bool Imu::init() {
  std::error_code ec;
  imu_.init(ec);

  if (ec) {
    return false;
  } else {
    return true;
  }
}

// whoami
uint8_t Imu::get_whoami() {
  std::error_code ec;
  return imu_.get_device_id(ec);
}

// update
bool Imu::update_raw(float dt) {
  std::error_code ec;
  if (!imu_.update(dt, ec)) {
    return false;
  }

  // store raw values
  raw_.accel = imu_.get_accelerometer();
  raw_.gyro = imu_.get_gyroscope();
  raw_.mag = imu_.get_magnetometer();
  raw_.temperature = imu_.get_temperature();

  return true;
}

bool Imu::update(float dt) {
  std::error_code ec;
  if (!imu_.update(dt, ec)) {
    return false;
  }

  // store raw values
  raw_.accel = imu_.get_accelerometer();
  raw_.gyro = imu_.get_gyroscope();
  raw_.mag = imu_.get_magnetometer();
  raw_.temperature = imu_.get_temperature();

  // apply magnetometer calibration
  auto calibrated_mag = calibrate_mag(raw_.mag);

  // apply madgwick filter
  filter_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
                 espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z), calibrated_mag.x,
                 calibrated_mag.y, calibrated_mag.z);
  // filter_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
  //                espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z));

  // get quaternion values
  filter_.get_quaternion(orientation_.w, orientation_.x, orientation_.y, orientation_.z);

  return true;
}

Imu::Value calibrate_mag(espp::icm20948::Value raw) {
  // Values generated during initial 1 time calibration from py script
  // Hard iron offset (bias)
  float b[3] = {3.795706, 15.694968, 25.832445};

  // Soft iron correction matrix (scale and cross-axis corrections)
  double A[3][3] = {
      {1.68545815e01, 4.21150134e-03, 1.623333486e-01},
      {4.21150134e-03, 1.59097586e01, -2.21327219e-02},
      {1.62333486e-01, -2.21327219e-02, 1.64177326e01},
  };

  double m_raw[3] = {raw.x, raw.y, raw.z};

  // Step 1: Subtract hard-iron offset
  double m_corr[3];
  for (int i = 0; i < 3; i++)
    m_corr[i] = m_raw[i] - b[i];

  // Step 2: Apply soft-iron correction matrix
  double m_cal[3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m_cal[i] += A[i][j] * m_corr[j];
    }
  }

  Imu::Value calibrated = {static_cast<float>(m_cal[0]), static_cast<float>(m_cal[1]),
                           static_cast<float>(m_cal[2])};

  return calibrated;
}
} // namespace SENSORS
