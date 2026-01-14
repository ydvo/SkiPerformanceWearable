#include "imu.hpp"
#include <system_error>

namespace SENSORS {

// constructor
Imu::Imu(espp::I2c &i2c) : Imu(make_default_config(i2c)) {}

Imu::Imu(ICM::Config cfg) : imu_(cfg) {}

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

  // // update Madgwick filter
  // madgwick_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
  //                  espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z), raw_.mag.x,
  //                  raw_.mag.y, raw_.mag.z);
  //
  // float roll_deg, pitch_deg, yaw_deg;
  // madgwick_.get_euler(roll_deg, pitch_deg, yaw_deg);
  //
  // // convert to radians for orientation
  // orientation_.roll = espp::deg_to_rad(roll_deg);
  // orientation_.pitch = espp::deg_to_rad(pitch_deg);
  // orientation_.yaw = espp::deg_to_rad(yaw_deg);

  return true;
}
} // namespace SENSORS
