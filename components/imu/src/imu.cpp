#include "imu.hpp"
#include "esp_log.h"
#include "fast_math.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include <system_error>

namespace SENSORS {

static const char *TAG{"IMU"};

// Forward declaration
Imu::Value calibrate_mag(espp::icm20948::Value raw);

// constructor
Imu::Imu(espp::I2c &i2c) : Imu(make_default_config(i2c)) {
  // Set the combined write_then_read callback on the base peripheral to reduce bus operations
  imu_.set_write_then_read(std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1,
                                     std::placeholders::_2, std::placeholders::_3,
                                     std::placeholders::_4, std::placeholders::_5));
}

Imu::Imu(ICM::Config cfg) : imu_(cfg), filter_(MADGWICK_BETA), enable_magnetometer_{1} {}

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

  // initialize imu
  if (!imu_.init(ec)) {
    ESP_LOGE(TAG, "Error initializing: %s", ec.message().c_str());
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(INIT_DELAY));

  // set and enable low pass filters
  if (!imu_.set_accelerometer_dlpf(ACCEL_LPF_BANDWIDTH, ec)) {
    ESP_LOGE(TAG, "Error setting accel lpf bandwidth: %s", ec.message().c_str());
    return false;
  }

  if (!imu_.set_accelerometer_dlpf_enabled(true, ec)) {
    ESP_LOGE(TAG, "Error enabling accel lpf: %s", ec.message().c_str());
    return false;
  }

  if (!imu_.set_gyroscope_dlpf(GYRO_LPF_BANDWIDTH, ec)) {
    ESP_LOGE(TAG, "Error setting gyro lpf bandwidth: %s", ec.message().c_str());
    return false;
  }

  if (!imu_.set_gyroscope_dlpf_enabled(true, ec)) {
    ESP_LOGE(TAG, "Error enabling gyro lpf: %s", ec.message().c_str());
    return false;
  }

  return true;
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
  auto calibrated_mag = apply_mag_cal(raw_.mag);

  // apply madgwick filter
  if (enable_magnetometer_) {
    filter_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
                   espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z), calibrated_mag.x,
                   calibrated_mag.y, calibrated_mag.z);
  } else {
    filter_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
                   espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z));
  }

  // get quaternion values
  filter_.get_quaternion(orientation_.w, orientation_.x, orientation_.y, orientation_.z);

  return true;
}

Imu::Euler Imu::get_euler() const {
  Euler e;
  filter_.get_euler(e.pitch, e.roll, e.yaw);
  return e;
}

Imu::Value Imu::apply_mag_cal(espp::icm20948::Value raw) {
  double m_raw[3] = {raw.x, raw.y, raw.z};

  // Step 1: Subtract hard-iron offset
  double m_corr[3];
  for (int i = 0; i < 3; i++)
    m_corr[i] = m_raw[i] - b_[i];

  // Step 2: Apply soft-iron correction matrix
  double m_cal[3] = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m_cal[i] += A_[i][j] * m_corr[j];
    }
  }

  // store value and orient axes
  Imu::Value calibrated = {static_cast<float>(m_cal[0]), -static_cast<float>(m_cal[1]),
                           -static_cast<float>(m_cal[2])};

  return calibrated;
}

} // namespace SENSORS
