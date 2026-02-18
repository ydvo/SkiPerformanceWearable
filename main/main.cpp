#include "flash_log.hpp"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "imu.hpp"

#include "driver/spi_common.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"

#include "GPIO.hpp"
#include "ble.hpp"
#include "flash.hpp"
#include "fsr.hpp"
#include "fuelgauge.hpp"
#include "haptic_motor.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "logger.hpp"

#include <cstddef>
#include <cstdio>
#include <stdint.h>

// ---------------------------------------------------------------------
//  Hardware Pin Definitions
// ---------------------------------------------------------------------

// I2C pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// fsr pin
constexpr adc_channel_t fsr_pin{ADC_CHANNEL_4};

// SPI2 pins (for flash storage)
static constexpr auto spi2_sck{GPIO_NUM_12};
static constexpr auto spi2_mosi{GPIO_NUM_11};
static constexpr auto spi2_miso{GPIO_NUM_13};
static constexpr auto flash_spi_cs{GPIO_NUM_10};

// ---------------------------------------------------------------------
// Setup for Control Flow
// ---------------------------------------------------------------------
// possible states
enum SYSTEM_STATES { SLEEP, SETUP, IDLE, READY, RUNNING, ERROR };
enum EVENTS {
  BLE_CONNECTED = BIT0,
  BLE_DISCONNECTED = BIT1,
  ACTIVE = BIT2,
  BLE_IDLE = BIT3,
  START_RUN = BIT4
};

// state var
static SYSTEM_STATES system_state;
static EventGroupHandle_t system_events;

// ---------------------------------------------------------------------
//  Global Objects
// ---------------------------------------------------------------------

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

// SPI Flash Device
STORAGE::SpiFlashDevice spi_flash({.host = SPI2_HOST, .cs = flash_spi_cs});

// Flash Log
STORAGE::FlashLog<STORAGE::Quaternion> flash_log(spi_flash);

// IMU
SENSORS::Imu imu(i2c);

// Power
POWER::fuelgauge battery(i2c);

// Forse sensor
SENSORS::fsr pressure_sensor(fsr_pin);

// Haptics
HAPTICS::DRV2605L haptic(i2c);

// BLE Module
BLE::BleModule ble;

Common::GPIO boot_button(GPIO_NUM_0, Common::GPIO::Direction::INPUT, Common::GPIO::Level::ON,
                         Common::GPIO::PULLUP);

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

// button debounce
constexpr uint32_t BOOT_BUTTON_DEBOUNCE_MS{200};

// force sensor calibration value defaults
constexpr float FSR_LOW{0.00f};
constexpr float FSR_HIGH{2893.5f};

// ring buffer size
constexpr uint8_t RINGBUFFER_SIZE{32};

// number of samples to send over ble
constexpr uint8_t NUM_SAMPlES_BLE{24};

// imu frequency
constexpr float IMU_FREQ{100};

// ---------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------

// ring buffer for imu data
RingbufHandle_t quatRing = nullptr;

// structure of data to be stored in buffer
struct quat_sample_t {
  SENSORS::Imu::Quaternion quat;
  int64_t timestamp_us;
};

// imu task
void imuTask(void *arg) {
  TickType_t lastWake;
  quat_sample_t s;

  for (;;) {
    // Wait until Running
    xEventGroupWaitBits(system_events, RUNNING, false, true, portMAX_DELAY);

    // get current time for scheduling
    lastWake = xTaskGetTickCount();

    while (xEventGroupGetBits(system_events) & RUNNING) {
      // save timestamp
      s.timestamp_us = esp_timer_get_time();

      // save imu data
      if (imu.update(IMU_FREQ * 1e-6)) {
        s.quat = imu.get_orientation();
      } else {
        s.quat = {1, 0, 0, 0}; // imu not ready, return identity quat
      }

      // push to buffer
      xRingbufferSend(quatRing, &s, sizeof(s), 0);

      // wait until 100Hz
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_FREQ));
    }
  }
}

// ---------------------------------------------------------------------
// ISRs
// ---------------------------------------------------------------------
static void IRAM_ATTR boot_button_isr(void *arg) {
  static bool prev_state = 0;

  // set or clear start signal
  if (prev_state) {
    xEventGroupClearBits(system_events, START_RUN);
  } else {
    xEventGroupSetBits(system_events, START_RUN);
  }
  // toggle state
  prev_state ^= prev_state;
}

// ---------------------------------------------------------------------
//  BLE
// ---------------------------------------------------------------------

BLE::FrameSample samples_to_send[NUM_SAMPlES_BLE] = {
    {1, 0.0, 0.0, 0.0, 0.0},  {2, 0.0, 0.0, 0.0, 0.0},  {3, 0.0, 0.0, 0.0, 0.0},
    {4, 0.0, 0.0, 0.0, 0.0},  {5, 0.0, 0.0, 0.0, 0.0},  {6, 0.0, 0.0, 0.0, 0.0},
    {7, 0.0, 0.0, 0.0, 0.0},  {8, 0.0, 0.0, 0.0, 0.0},  {9, 0.0, 0.0, 0.0, 0.0},
    {10, 0.0, 0.0, 0.0, 0.0}, {11, 0.0, 0.0, 0.0, 0.0}, {12, 0.0, 0.0, 0.0, 0.0},
    {13, 0.0, 0.0, 0.0, 0.0}, {14, 0.0, 0.0, 0.0, 0.0}, {15, 0.0, 0.0, 0.0, 0.0},
    {16, 0.0, 0.0, 0.0, 0.0}, {17, 0.0, 0.0, 0.0, 0.0}, {18, 0.0, 0.0, 0.0, 0.0},
    {19, 0.0, 0.0, 0.0, 0.0}, {20, 0.0, 0.0, 0.0, 0.0}, {21, 0.0, 0.0, 0.0, 0.0},
    {22, 0.0, 0.0, 0.0, 0.0}, {23, 0.0, 0.0, 0.0, 0.0}, {24, 0.0, 0.0, 0.0, 0.0},
};

// callbacks to change system state on connection status
void on_ble_connect(NimBLEConnInfo &info) {
  xEventGroupSetBits(system_events, BLE_CONNECTED);
  xEventGroupClearBitsFromISR(system_state, BLE_DISCONNECTED);
  logger.info("BLE: Client connected \n");
}

void on_ble_disconnect(NimBLEConnInfo &info, espp::BleGattServer::DisconnectReason reason) {
  xEventGroupSetBits(system_events, BLE_DISCONNECTED);
  xEventGroupClearBitsFromISR(system_state, BLE_CONNECTED);
  logger.info("BLE: Client disconnected \n");
}

esp_err_t send_ble() {
  // Timing state
  static int64_t last_send_time_us = 0;
  static int64_t last_state_log_us = 0;
  static int64_t last_battery_update_us = 0;
  static int64_t last_waiting_log_us = 0;
  static uint16_t frame_seq = 0;
  static size_t sample_idx_to_send = 0;

  static bool was_connected = false;

  const int64_t ACK_TIMEOUT_US = 30'000'000; // 30 seconds

  int64_t now_us = esp_timer_get_time();
  bool is_connected = ble.is_connected();

  if (!is_connected) {
    // check if first connection
    if (was_connected) {
      logger.info("Device disconnected – resetting state");
      was_connected = false;
    }

    // prompt for connection
    if (now_us - last_waiting_log_us >= 10'000'000) { // Every 10 seconds
      last_waiting_log_us = now_us;
      logger.info("Waiting for BLE connection…");
    }

    return ESP_OK;
  }

  // on first connection
  if (!was_connected) {
    logger.info("Device connected! Getting device info…");
    auto devs = ble.get_connected_device_infos();
    for (const auto &d : devs) {
      logger.info("Device: {}, RSSI: {} dBm", ble.get_device_name(d), ble.get_rssi(d));
    }
    was_connected = true;
    ble.reset_ack_on_connect();
    last_send_time_us = now_us;
  }

  // ACK/timeout handling
  bool can_send = ble.is_ack_received();
  bool timeout = (now_us - last_send_time_us) >= ACK_TIMEOUT_US;

  // Periodic status logging (every 5 seconds)
  if (now_us - last_state_log_us >= 5'000'000) {
    last_state_log_us = now_us;
    int64_t secs_since_last = (now_us - last_send_time_us) / 1'000'000;
    if (can_send) {
      logger.info("Ready to send (ACK received or first-send pending)");
    } else {
      logger.info("Waiting for ACK – {} s since last send", secs_since_last);
    }
  }

  // ---------------------------------------------------------------------
  // SEND LOOP
  // ---------------------------------------------------------------------
  while (can_send || timeout) {
    // We have a frame - pack into bulk format
    uint16_t sample_count = NUM_SAMPlES_BLE - sample_idx_to_send >= BLE::SAMPLES_PER_FRAME
                                ? BLE::SAMPLES_PER_FRAME
                                : NUM_SAMPlES_BLE - sample_idx_to_send;

    // length of transmission: samples * frame_size
    uint16_t payload_len = sample_count * ((uint16_t)sizeof(BLE::FrameSample));

    // frame to send over ble
    BLE::Frame frame{.header =
                         {
                             .frame_seq = frame_seq++,
                             .sample_count = sample_count,
                             .payload_len = payload_len,
                             .flags = 0,
                         },
                     .payload = {}};

    memcpy(frame.payload, &samples_to_send[sample_idx_to_send], payload_len);

    logger.info("Sending bulk frame seq {} (244 B)", (unsigned int)frame.header.frame_seq);
    ble.notify_quaternion(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
    ble.reset_ack();
    last_send_time_us = now_us;

    sample_idx_to_send = sample_count + sample_idx_to_send >= NUM_SAMPlES_BLE
                             ? 0
                             : sample_count + sample_idx_to_send;

    // Re-evaluate send condition
    can_send = ble.is_ack_received();
    timeout = false;
  }

  // ---------------------------------------------------------------------
  // Battery update (every 30 seconds)
  // ---------------------------------------------------------------------
  if (now_us - last_battery_update_us >= 30'000'000) {
    last_battery_update_us = now_us;
    float battery_level = battery.cellPercent();
    ble.set_battery_level(battery_level);
    logger.info("Battery level: {}%", battery_level);
  }

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  System Initialization
// ---------------------------------------------------------------------

esp_err_t initSystem() {
  logger.info("Initializing...");

  // enable QT Stemma Port
  Common::GPIO stemma_qt_power =
      Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, Common::GPIO::Level::ON);
  logger.info("Enabled QT Stemma Port");

  // Attach interrupt to boot button
  ESP_RETURN_ON_ERROR(
      boot_button.set_interrupt(Common::GPIO::INTERRUPT_FALLING_EDGE, boot_button_isr, nullptr),
      "SYS_INIT", "Failed to set interrupt for boot button.");
  logger.info("Enabled Boot Button");

  // Initialize LED
  LED::led red_led = LED::led(LED::RED_LED);

  // create i2c instance
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);
  std::error_code ec;
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
    return ESP_ERR_INVALID_STATE;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // give i2c time to startup

  // init imu
  if (imu.init()) {
    logger.info("Imu initialized");
  } else {
    logger.error("Failed to initialize imu");
    // return ESP_ERR_INVALID_STATE; // sometimes errors but works fine
  }

  // check if battery is connected
  if (!battery.isDeviceReady())
    logger.error("Could not initialize fuel gauge. Check battery connection");

  // setup force sensor
  pressure_sensor.set_calibration(FSR_LOW, FSR_HIGH);

  // Haptic setup
  if (haptic.init()) {

    // set waveform
    haptic.set_waveform(0, HAPTICS::DRV2605L::EFFECTS::DOUBLE_CLICK);
    haptic.set_waveform(1, 0); // end sequence

    // Trigger the waveform
    haptic.go();

    // Wait for effect to complete
    vTaskDelay(pdMS_TO_TICKS(250));

  } else {
    logger.error("Could not initialize haptics");
    return ESP_ERR_INVALID_STATE;
  }

  // initialize spi
  //  - might want to move in own module
  spi_bus_config_t spi2_bus_config{
      .mosi_io_num = spi2_mosi,
      .miso_io_num = spi2_miso,
      .sclk_io_num = spi2_sck,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };

  ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &spi2_bus_config, SPI_DMA_CH_AUTO), "SYS_INIT",
                      "Failed to initialize SPI_2 bus.");

  // init flash
  ESP_RETURN_ON_ERROR(spi_flash.init(), "SYS_INIT", "Failed to initialize SPI flash.");
  ESP_RETURN_ON_ERROR(flash_log.init(), "SYS_INIT", "Failed to initialize flash log.");

  // init ring buffer
  quatRing = xRingbufferCreate(RINGBUFFER_SIZE * sizeof(quat_sample_t), RINGBUF_TYPE_NOSPLIT);

  if (quatRing == nullptr) {
    ESP_LOGE("SYS_INIT", "Failed to create flash writer buffer.");
    return ESP_ERR_INVALID_STATE;
  }

  // Configure BLE
  BLE::BleModule::Config ble_config;
  ble_config.device_name = "Ski Wearable Test";
  ble_config.manufacturer_name = "ESP-CPP";
  ble_config.model_number = "ski-wearable-01";
  ble_config.serial_number = "TEST123456";

  // BLE connection callbacks
  ble_config.on_connect = on_ble_connect;
  ble_config.on_disconnect = on_ble_disconnect;
  ble_config.on_authenticated = [](const NimBLEConnInfo &info) {
    logger.info("BLE: Client authenticated\n");
  };

  // Create and initialize BLE module
  ble = BLE::BleModule(ble_config);
  ESP_RETURN_ON_ERROR(ble.init(), "SYS_INIT", "Failed to initialize BLE module");

  // Set MTU for larger packets
  ESP_RETURN_ON_FALSE(NimBLEDevice::setMTU(247), ESP_ERR_INVALID_STATE, "SYS_INIT",
                      "Failed to set MTU on NimBLEDevice.");

  logger.info("BLE module initialized successfully");

  // Start advertising
  ESP_RETURN_ON_ERROR(ble.start_advertising(), "SYS_INIT", "Failed to start advertising");

  logger.info("BLE advertising started. Device name: {}", ble_config.device_name);
  logger.info("Connect with your phone's BLE scanner app");

  // turn on led to indicate successful init
  red_led.turn_on();

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  Application Entry Point
// ---------------------------------------------------------------------

extern "C" void app_main() {
  // Setup
  if (initSystem() != ESP_OK) {
    logger.error("Failed initializing a system.");
  } // called once

  // Create Eventgroup
  system_events = xEventGroupCreate();

  // Create Tasks
  // Create a task that will write samples to the flash
  xTaskCreate(flash_writer_task, "flash_writer", 4096, NULL, 3, &flash_writer_task_handle);

  // Create a task that will capture sensor data
  xTaskCreate(capture_sensor_data_task, "capture_sensor_data", 4096, NULL, 8, &capture_task_handle);

  // Create a task that will control program flow
  xTaskCreate(control_task, "control", 4096, NULL, 9, &control_task_handle);

  // Create a task that will transmit data
  xTaskCreate(upload_task, "upload", 4096, NULL, 5, &upload_task_handle);

  // start system in idle
  system_state = idle;
}
