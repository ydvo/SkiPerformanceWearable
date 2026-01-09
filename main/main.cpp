#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_log.h"
#include "esp_err.h"

#include "logger.hpp"
#include "i2c.hpp"
#include "icm20948.hpp"

#include <cstdio>

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr gpio_num_t sda{GPIO_NUM_3};
constexpr gpio_num_t scl{GPIO_NUM_4};

/* Main function */
extern "C" void app_main() {
  using Imu = espp::Icm20948<espp::icm20948::Interface::I2C>; 

  espp::Logger logger({.tag = "Ski-wearable module", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  // enable QT Stemma Port
  // TODO: Add proper error handling, abort for now. 
  gpio_num_t vcc = GPIO_NUM_7;
  ESP_ERROR_CHECK(gpio_set_direction(vcc, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_level(vcc, 1));

  static constexpr auto i2c_port {I2C_NUM_0}; 
  static constexpr auto i2c_sda {GPIO_NUM_3};
  static constexpr auto i2c_scl {GPIO_NUM_4}; 
  
  logger.info("Enabled QT Stemma Port");
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);

  espp::I2c i2c({
    .port = i2c_port,
    .sda_io_num = i2c_sda,
    .scl_io_num = i2c_scl,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
  });

  logger.info("Scanning I2C devices"); 
  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 1; address < 128; address++) {
    if (i2c.probe_device(address)) {
      found_addresses.push_back(address);
    }
  }
  logger.info("Found devices at addresses: {::#02x}", found_addresses);

  static constexpr uint8_t imu_address {ICM20948_ADRESS};
  Imu::Config imu_config {
    .device_address = imu_address, 
    .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
    .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), 
    .imu_config = {
      .accelerometer_range = Imu::AccelerometerRange::RANGE_2G, 
      .gyroscope_range = Imu::GyroscopeRange::RANGE_250DPS, 
      .accelerometer_sample_rate_divider = 9, // 1kHz / (1 + 9) = 100Hz
      .gyroscope_sample_rate_divider = 9,     // 1kHz / (1 + 9) = 100Hz
      .magnetometer_mode = Imu::MagnetometerMode::CONTINUOUS_MODE_100_HZ
    }
  }; 

  logger.info("Creating IMU"); 
  Imu imu {imu_config}; 

  using namespace std::chrono_literals;
  // Main event loop
  while (true) {
    auto now {std::chrono::system_clock::now()}; 
    static auto t0 {now}; 
    auto t1 {now}; 
    std::chrono::duration<float> dt_ = t1 - t0;
    float dt = dt_.count(); 
    t0 = t1;
    logger.info("Elapsed time in float seconds: {}", dt);

    std::error_code ec;
    // update the imu data
    if (!imu.update(dt, ec)) {
      logger.error("Failed to update IMU: {}", ec.message());
    }

    auto accel = imu.get_accelerometer();
    auto gyro = imu.get_gyroscope();
    auto mag = imu.get_magnetometer();

    logger.info("Imu Acceleration: < {} , {} , {} > ", accel.x, accel.y, accel.z);
    logger.info("Imu Gyroscope: < {} , {} , {} > ", gyro.x, gyro.y, gyro.z);
    logger.info("Imu Magnetometer: < {} , {} , {} > ", mag.x, mag.y, mag.z);

    std::this_thread::sleep_for(1s);
  }
}
