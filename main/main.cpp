#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_err.h"

#include "GPIO.hpp"
#include "fuelgauge.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"

#include "fsr.hpp"
#include "haptic_motor.hpp"

#include <cstdio>
#include <stdint.h>

/* Constants */

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// fsr pin
constexpr adc_channel_t fsr_pin{ADC_CHANNEL_4};

/* Components */

// Logging
espp::Logger logger({.tag = "MAIN", .level = espp::Logger::Verbosity::INFO});

// I2C
espp::I2c i2c({
    .port = i2c_port,
    .sda_io_num = i2c_sda,
    .scl_io_num = i2c_scl,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .auto_init = false,
});

// IMU
SENSORS::Imu imu(i2c);

// Power
POWER::fuelgauge battery(i2c);

// Forse sensor
SENSORS::fsr pressure_sensor(fsr_pin);

// Haptics
HAPTICS::DRV2605L haptic(i2c);

/* Flags */
bool force_sensor_pressed = false;

/*
 * initSystem()
 *  - initialization function, runs once before main loop
 */
void initSystem() {

  logger.info("Initializing...");

  // enable QT Stemma Port
  Common::GPIO stemma_qt_power =
      Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, Common::GPIO::Level::ON);
  logger.info("Enabled QT Stemma Port");

  // led
  LED::led red_led = LED::led(LED::RED_LED);

  // turn on led
  red_led.turn_on();

  // create i2c instance
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);

  std::error_code ec;
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // give i2c time to startup

  // init imu
  bool imu_initialized = imu.init();
  // ensure imu is configured correctly

  vTaskDelay(pdMS_TO_TICKS(500)); // give imu time to startup before first i2c read

  if (imu.get_whoami() == 0xEA && imu_initialized) {
    logger.info("Imu initialized");
  }

  // // for testing
  // imu.disable_magnetometer();

  // check if battery is connected
  if (!battery.isDeviceReady())
    logger.error("Could not initialize fuel gauge. Check battery connection");

  // setup force sensor
  pressure_sensor.set_calibration(0.00f, 2893.5f);

  // Haptic setup
  if (haptic.init()) {

    // Set to internal trigger mode
    haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);

    // Select ERM motor
    haptic.select_motor(HAPTICS::DRV2605L::MotorType::ERM);

    // Select library
    haptic.select_library(HAPTICS::DRV2605L::Library::ERM_LIB_A);

    // Set waveform sequence
    // haptic.set_waveform(0, HAPTICS::DRV2605L::EFFECTS::DOUBLE_CLICK); // double click
    // haptic.set_waveform(1, HAPTICS::DRV2605L::EFFECTS::DOUBLE_CLICK); // double click
    // haptic.set_waveform(2, HAPTICS::DRV2605L::EFFECTS::LONG_BUZZ);    // buzz
    // haptic.set_waveform(3, 0);                                        // End of sequence
    haptic.set_waveform(0, HAPTICS::DRV2605L::EFFECTS::DOUBLE_CLICK);
    haptic.set_waveform(1, 0);

    // Trigger the waveform
    haptic.go();

    // Set threshold to 50%
    float threshold = 0.5f;
    threshold = pressure_sensor.get_baseline() +
                (pressure_sensor.get_max() - pressure_sensor.get_baseline()) * threshold;
    pressure_sensor.set_pressure_threshold(threshold);

    // Wait for effect to complete
    vTaskDelay(pdMS_TO_TICKS(500));

    haptic.set_mode(HAPTICS::DRV2605L::Mode::REALTIME_PLAYBACK);
    haptic.set_realtime_value(0);
  } else {
    logger.error("Could not initialize haptics");
  }
}

/*
 * mainLoop
 *  - runs repeatedly, contains update logic
 */
void mainLoop(auto dt) {

  // check fsr
  auto force_percent = pressure_sensor.read_percentage();

  // get imu data
  if (imu.update(dt)) {
    SENSORS::Imu::Quaternion quat = imu.get_orientation();

    // read from fsr

    printf("DATA %0.4f %0.4f %0.4f %0.4f %0.4f\r\n", quat.w, quat.x, quat.y, quat.z, force_percent);
  }
  // float bat_volt = battery.cellVoltage();
  // float bat_percent = battery.cellPercent();
  // float discharge = battery.chargeRate();
  //
  // printf("Battery state: %0.4fV %0.4f %0.4f per hour\r\n", bat_volt, bat_percent, discharge);

  // haptic if above threshold
  if (pressure_sensor.is_pressed()) {
    haptic.set_realtime_value(127);
  } else {
    haptic.set_realtime_value(0);
  }
}

/* Application Entry Point */
extern "C" void app_main() {
  initSystem(); // called once

  // Main event loop
  while (true) {
    // get elapsed time in between loops
    auto now{std::chrono::system_clock::now()};
    static auto t0{now};
    auto t1{now};
    std::chrono::duration<float> dt_ = t1 - t0;
    auto dt = dt_.count();
    t0 = t1;
    // logger.info("Elapsed time in float seconds: {}", dt);

    mainLoop(dt); // run repeatedly

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
