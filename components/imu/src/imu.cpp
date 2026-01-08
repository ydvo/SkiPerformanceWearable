#include "imu.hpp"

namespace SENSORS {

// constructor
Imu::Imu(Common::I2C &i2c) : Imu(make_default_config(i2c)) {}

Imu::Imu(const Config &config)
    : imu_(espp::Icm20948<espp::icm20948::Interface::I2C>::Config{
          config.device_address, config.write, config.read, config.imu_config,
          nullptr, // orientation_filter
          config.auto_init}),
      madgwick_(config.madgwick_beta) {}

// Construct default config from i2c
Imu::Config Imu::make_default_config(Common::I2C &i2c) {
  uint8_t dev_addr = ICM20948_ADDRESS;

  return {.device_address = dev_addr,
          .write = [&i2c](uint8_t reg, const uint8_t *data, size_t len) -> bool {
            std::error_code ec;
            return i2c.espp_write(ICM20948_ADDRESS, reg, data, len, ec);
          },
          .read = [&i2c](uint8_t reg, uint8_t *data, size_t len) -> bool {
            std::error_code ec;
            return i2c.espp_read(ICM20948_ADDRESS, reg, data, len, ec);
          },
          .imu_config =
              {
                  .accelerometer_range = ACCELEROMETER_RANGE,
                  .gyroscope_range = GYROSCOPE_RANGE,
                  .accelerometer_sample_rate_divider = 9,
                  .gyroscope_sample_rate_divider = 9,
                  .magnetometer_mode = MAGNETOMETER_MODE,
              },
          .madgwick_beta = MADGWICK_BETA,
          .auto_init = true};
}

// whoami
uint8_t Imu::get_whoami() {
  std::error_code ec;
  return imu_.get_device_id(ec);
}

// // update
// bool Imu::update(float dt) {
//   std::error_code ec;
//   if (!imu_.update(dt, ec)) {
//     return false;
//   }
//
//   // store raw values
//   raw_.accel = imu_.get_accelerometer();
//   raw_.gyro = imu_.get_gyroscope();
//   raw_.mag = imu_.get_magnetometer();
//   raw_.temperature = imu_.get_temperature();
//
//   // update Madgwick filter
//   madgwick_.update(dt, raw_.accel.x, raw_.accel.y, raw_.accel.z, espp::deg_to_rad(raw_.gyro.x),
//                    espp::deg_to_rad(raw_.gyro.y), espp::deg_to_rad(raw_.gyro.z), raw_.mag.x,
//                    raw_.mag.y, raw_.mag.z);
//
//   float roll_deg, pitch_deg, yaw_deg;
//   madgwick_.get_euler(roll_deg, pitch_deg, yaw_deg);
//
//   // convert to radians for orientation
//   orientation_.roll = espp::deg_to_rad(roll_deg);
//   orientation_.pitch = espp::deg_to_rad(pitch_deg);
//   orientation_.yaw = espp::deg_to_rad(yaw_deg);
//
//   return true;
// }
} // namespace SENSORS
