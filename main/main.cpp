#include "NimBLELocalValueAttribute.h"
#include "flash_log.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "imu.hpp"

#include "driver/spi_common.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/queue.h"
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
#include <string>

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
static constexpr uint32_t BOOT_BUTTON_DEBOUNCE_MS{200};

// force sensor calibration value defaults
static constexpr float FSR_LOW{0.00f};
static constexpr float FSR_HIGH{2893.5f};

// queue size for imu samples
static constexpr uint8_t IMU_QUEUE_SIZE{64};

// number of samples to batch for flash writes
static constexpr uint8_t FLASH_BATCH_SIZE{14};

// number of samples to send over ble
static constexpr uint8_t NUM_SAMPlES_BLE{24};

// upload task timing
static constexpr uint32_t UPLOAD_RETRY_DELAY_MS{100};
static constexpr int64_t UPLOAD_ACK_TIMEOUT_US{5'000'000}; // 5 seconds

// battery task timing
static constexpr uint32_t BATTERY_UPDATE_INTERVAL_MS{30'000}; // 30 seconds

// imu frequency
static constexpr float IMU_FREQ{100};

// length of event queue
static constexpr uint8_t EVENT_QUEUE_LENGTH{8};

// haptic queue length - small, events are low-frequency
static constexpr uint8_t HAPTIC_QUEUE_LENGTH{4};

enum class HapticEvent : uint8_t {
  BOOT,           // device powered on and init succeeded
  BLE_CONNECT,    // client connected
  BLE_DISCONNECT, // client disconnected
  RUN_START,      // recording started
  RUN_STOP,       // recording stopped
  LOW_BATTERY,    // battery below threshold
  ERROR,          // unrecoverable error
};

// ---------------------------------------------------------------------
// Setup for Control Flow
// ---------------------------------------------------------------------
// possible states
enum class SystemState : EventBits_t {
  SLEEP = BIT0,
  IDLE = BIT1,
  READY = BIT2,
  RUNNING = BIT3,
  ERROR = BIT4,
};

// state var
static EventGroupHandle_t system_state;

// events
enum class Event : uint8_t {
  BLE_CONNECTED,
  BLE_DISCONNECTED,
  TOGGLE_RUN,
};

// statically allocated event queue
static Event eventQueueStorage[EVENT_QUEUE_LENGTH];
static StaticQueue_t eventQueueStruct;
static QueueHandle_t eventQueue;

// ---------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------

// Task handles
static TaskHandle_t imu_task_handle = nullptr;
static TaskHandle_t flash_writer_task_handle = nullptr;
static TaskHandle_t control_task_handle = nullptr;
static TaskHandle_t upload_task_handle = nullptr;
static TaskHandle_t battery_task_handle = nullptr;
static TaskHandle_t haptic_task_handle = nullptr;

// queue for imu data
static QueueHandle_t imuQueue = nullptr;

// queue for haptic events
static QueueHandle_t hapticQueue = nullptr;

// Enqueue a haptic event
static inline void haptic_notify(HapticEvent event) {
  if (hapticQueue != nullptr) {
    xQueueSend(hapticQueue, &event, 0); // non-blocking, drop if queue full
  }
}

// structure of data to be stored in sensor data queue
struct quat_sample_t {
  uint64_t timestamp_us;
  SENSORS::Imu::Quaternion quat;
};

// Capture sensor data and push to queue
void imuTask(void *arg) {
  TickType_t lastWake;
  quat_sample_t sample;

  for (;;) {
    // Wait until Running
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    // get current time for scheduling
    lastWake = xTaskGetTickCount();

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {
      // save timestamp
      sample.timestamp_us = esp_timer_get_time();

      // save imu data
      if (imu.update(1.0f / IMU_FREQ)) {
        sample.quat = imu.get_orientation();
      } else {
        sample.quat = {1, 0, 0, 0}; // imu not ready, return identity quat
      }

      // push to queue (non-blocking, drop if full)
      if (xQueueSend(imuQueue, &sample, 0) != pdTRUE) {
        logger.warn("IMU queue full, sample dropped");
      }

      // wait until imu frequency (100Hz)
      xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_FREQ));
    }
  }
}

// Batch read from queue and write to flash
void flashWriterTask(void *arg) {
  quat_sample_t sample;

  for (;;) {
    // Wait until Running
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    // TODO: Is there a need to reinitialize flash log?

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {
      // Block waiting for first sample
      if (xQueueReceive(imuQueue, &sample, portMAX_DELAY) == pdTRUE) {
        // Write batch to flash
        STORAGE::Quaternion q = {
            .w = sample.quat.w,
            .x = sample.quat.x,
            .y = sample.quat.y,
            .z = sample.quat.z,
        };

        if (flash_log.append(q, sample.timestamp_us) != ESP_OK) {
          logger.error("Failed to append sample to flash log");
        }
      }
    }
  }
}

enum class UploadState : uint8_t {
  BUFFER_FLASH_FRAME,
  INITIALIZE_BLE_FRAME,
  NOTIFY_BLE,
  WAIT_FOR_ACK,
  RECEIVED_ACK,
  TIMEOUT_ACK,
};

// Read stored samples from flash and send over BLE
void uploadTask(void *arg) {
  uint32_t total_frames_sent = 0;
  uint32_t total_ack_timeouts = 0;

  for (;;) {
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    UploadState upload_state = UploadState::BUFFER_FLASH_FRAME;

    STORAGE::FlashLog<STORAGE::Quaternion>::Frame flash_frame;
    BLE::Frame ble_frame{
        .header = {.frame_seq = 0, .sample_count = 0, .payload_len = 0, .flags = 0}, .payload = {}};

    size_t samples_sent_from_frame{0};
    int64_t ble_ack_start;
    int64_t ack_wait_us;

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {
      switch (upload_state) {
      case UploadState::BUFFER_FLASH_FRAME: {
        size_t frames_read{0};

        logger.debug("Upload: reading frame from flash");
        if (flash_log.read(&flash_frame, 1, &frames_read) == ESP_OK && frames_read > 0) {
          samples_sent_from_frame = 0;
          logger.info("Upload: loaded flash frame seq={} ({} samples per frame)",
                      (uint32_t)flash_frame.seq, (uint32_t)STORAGE::SAMPLES_PER_FRAME);

          upload_state = UploadState::INITIALIZE_BLE_FRAME;
        } else {
          logger.debug("Upload: no flash data, retrying in {}ms", UPLOAD_RETRY_DELAY_MS);
          vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
        }

        break;
      }
      case UploadState::INITIALIZE_BLE_FRAME: {
        size_t samples_remaining = STORAGE::SAMPLES_PER_FRAME - samples_sent_from_frame;
        size_t samples_this_send = (samples_remaining > BLE::SAMPLES_PER_FRAME)
                                       ? BLE::SAMPLES_PER_FRAME
                                       : samples_remaining;

        ble_frame.header.frame_seq++;
        ble_frame.header.sample_count = static_cast<uint16_t>(samples_this_send);
        ble_frame.header.payload_len =
            static_cast<uint16_t>(samples_this_send * sizeof(BLE::FrameSample));

        for (size_t i = 0; i < samples_this_send; i++) {
          size_t src_idx = samples_sent_from_frame + i;
          ble_frame.payload[i].timestamp = flash_frame.payload[src_idx].timestamp_us;
          ble_frame.payload[i].w = flash_frame.payload[src_idx].data.w;
          ble_frame.payload[i].x = flash_frame.payload[src_idx].data.x;
          ble_frame.payload[i].y = flash_frame.payload[src_idx].data.y;
          ble_frame.payload[i].z = flash_frame.payload[src_idx].data.z;
        }

        upload_state = UploadState::NOTIFY_BLE;
        break;
      }
      case UploadState::NOTIFY_BLE: {
        // Wait for BLE connection
        if (!ble.is_connected()) {
          logger.debug("Upload: BLE not connected, retrying in {}ms", UPLOAD_RETRY_DELAY_MS);
          vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
          break;
        }

        size_t payload_bytes = sizeof(BLE::FrameHeader) + ble_frame.header.payload_len;

        logger.info("Upload: sending frame seq={} samples={} offset={} payload_bytes={}",
                    (uint16_t)ble_frame.header.frame_seq, (uint16_t)ble_frame.header.sample_count,
                    (uint32_t)samples_sent_from_frame, (uint32_t)payload_bytes);

        // Send notification
        esp_err_t notify_err =
            ble.notify_quaternion(reinterpret_cast<const uint8_t *>(&ble_frame), payload_bytes);

        if (notify_err != ESP_OK) {
          logger.warn("Upload: notify_quaternion failed (err={}), retrying in {}ms",
                      (int)notify_err, UPLOAD_RETRY_DELAY_MS);
          vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
        } else {
          // Arm the ACK wait
          ble.arm_ack(ble_frame.header.frame_seq);

          logger.debug("Upload: notification sent, armed ACK wait for seq={}",
                       (uint16_t)ble_frame.header.frame_seq);

          upload_state = UploadState::WAIT_FOR_ACK;
          ble_ack_start = esp_timer_get_time();
        }

        break;
      }
      case UploadState::WAIT_FOR_ACK: {
        ack_wait_us = esp_timer_get_time() - ble_ack_start;

        if (ble.is_ack_received()) {
          upload_state = UploadState::RECEIVED_ACK;
          break;
        }

        if (ack_wait_us > UPLOAD_ACK_TIMEOUT_US) {
          upload_state = UploadState::TIMEOUT_ACK;
          break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        break;
      }
      case UploadState::RECEIVED_ACK: {
        logger.info("Upload: ACK received for seq={} after {:.1f}ms",
                    (uint16_t)ble_frame.header.frame_seq, (float)ack_wait_us / 1000.0f);

        // Advance send position
        samples_sent_from_frame += ble_frame.header.sample_count;

        logger.debug("Upload: frame seq={} progress {}/{} samples",
                     (uint16_t)ble_frame.header.frame_seq, (uint32_t)samples_sent_from_frame,
                     (uint32_t)STORAGE::SAMPLES_PER_FRAME);

        if (samples_sent_from_frame >= STORAGE::SAMPLES_PER_FRAME) {
          total_frames_sent++;
          logger.info("Upload: flash frame seq={} fully transmitted (total frames sent={})",
                      (uint32_t)flash_frame.seq, total_frames_sent);

          upload_state = UploadState::BUFFER_FLASH_FRAME;
        } else {
          upload_state = UploadState::INITIALIZE_BLE_FRAME;
        }

        break;
      }
      case UploadState::TIMEOUT_ACK: {
        total_ack_timeouts++;
        logger.warn("Upload: ACK timeout after {:.2f}s for seq={} (total timeouts={})",
                    (float)ack_wait_us / 1e6f, (uint16_t)ble_frame.header.frame_seq,
                    total_ack_timeouts);
        upload_state = UploadState::NOTIFY_BLE;
        break;
      }
      default:
        logger.warn("Unexpected upload state. ");
        break;
      }
    }
  }
}

// Effect sequence table
namespace {
using E = HAPTICS::DRV2605L::EFFECTS;

struct HapticSequence {
  HapticEvent event;
  size_t count;
  uint8_t effects[8]; // terminated by EFFECTS::END (0x00)
};

static const HapticSequence haptic_sequences[] = {
    // BOOT: soft double-click to confirm power-on
    {HapticEvent::BOOT, 1, {E::DOUBLE_CLICK, E::END}},

    // BLE_CONNECT: rising two-pulse -- distinct from boot
    {HapticEvent::BLE_CONNECT, 2, {E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK_2, E::END}},

    // BLE_DISCONNECT: single dull bump to signal loss
    {HapticEvent::BLE_DISCONNECT, 1, {E::SOFT_BUMP, E::END}},

    // RUN_START: sharp strong click -- clear "go" signal
    {HapticEvent::RUN_START, 1, {E::SHARP_CLICK, E::END}},

    // RUN_STOP: two soft clicks -- distinct from start
    {HapticEvent::RUN_STOP, 2, {E::SOFT_BUMP, E::SOFT_BUMP_2, E::END}},

    // LOW_BATTERY: three short buzzes as a warning pattern
    {HapticEvent::LOW_BATTERY,
     3,
     {E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK, E::END}},

    // ERROR: long buzz
    {HapticEvent::ERROR, 1, {E::LONG_BUZZ, E::END}},
};
} // namespace

// Dequeue HapticEvents and play the corresponding effect sequence
void hapticTask(void *arg) {
  HapticEvent event;
  for (;;) {
    // Block indefinitely until an event is enqueued
    if (xQueueReceive(hapticQueue, &event, portMAX_DELAY) != pdTRUE)
      continue;

    // Find the matching sequence
    const HapticSequence *seq = nullptr;
    for (const auto &s : haptic_sequences) {
      if (s.event == event) {
        seq = &s;
        break;
      }
    }

    if (seq == nullptr) {
      logger.warn("hapticTask: unknown event {}", static_cast<uint8_t>(event));
      continue;
    }

    // Count entries up to (but not including) the terminator
    uint8_t count = seq->count;
    if (count > 0) {
      if (haptic.play(seq->effects, count)) {
        logger.debug("hapticTask: played event {}", static_cast<uint8_t>(event));
      } else {
        logger.warn("hapticTask: I2C write failed for event {}", static_cast<uint8_t>(event));
      }
    }
  }
}

// Periodically read fuel gauge and push battery level to BLE battery service
void batteryTask(void *arg) {
  for (;;) {
    if (ble.is_connected()) {
      uint8_t level = static_cast<uint8_t>(battery.cellPercent());
      if (ble.set_battery_level(level) != ESP_OK) {
        logger.warn("Battery: failed to update BLE battery level");
      } else {
        logger.debug("Battery: {}%", level);
      }
    }
    // check stack usage
    // UBaseType_t watermark = uxTaskGetStackHighWaterMark(nullptr);
    // logger.info("BatteryTask watermark: {}", watermark);
    vTaskDelay(pdMS_TO_TICKS(BATTERY_UPDATE_INTERVAL_MS));
  }
}

// ---------------------------------------------------------------------
// State Machine
// ---------------------------------------------------------------------

// idle mode callback to sleep everything and start ble_advertising
esp_err_t advertiseBLE() {
  /* TODO:
   *  - sleep imu
   *  - sleep fsr
   *  - haptics off prob
   */

  // Start advertising
  ESP_RETURN_ON_ERROR(ble.start_advertising(), "BLE", "Failed to start advertising");

  logger.info("BLE advertising started. Device name: {}", ble.get_name());
  logger.info("Connect with your phone's BLE scanner app");

  return ESP_OK;
}

esp_err_t startSensors() {
  /* TODO:
   *  - wake imu
   *  - wake fsr
   *  - start haptics
   */

  return ESP_OK;
}

// helper to transition states
static void switch_states(SystemState currstate, SystemState nextstate) {
  // set next state
  xEventGroupSetBits(system_state, static_cast<EventBits_t>(nextstate));
  // clear previous state
  xEventGroupClearBits(system_state, static_cast<EventBits_t>(currstate));

  // print state for debug
  std::string s;
  switch (nextstate) {
  case SystemState::SLEEP:
    s = "SLEEP";
    break;

  case SystemState::IDLE:
    s = "IDLE";
    break;

  case SystemState::READY:
    s = "READY";
    break;

  case SystemState::RUNNING:
    s = "RUNNING";
    break;
  default:
    s = "ERROR";
    break;
  }
  logger.info("State: {}", s);
}

// state machine
//  - transitions on Events
void controlTask(void *arg) {
  Event e;
  while (true) {
    // wait for events to transition states
    if (xQueueReceive(eventQueue, &e, portMAX_DELAY)) {

      // get current state
      SystemState currstate = static_cast<SystemState>(xEventGroupGetBits(system_state));

      // transition between states
      switch (currstate) {
      case SystemState::SLEEP:

        // TODO: sleep state
        break;

      case SystemState::IDLE: // waiting for ble connection
        switch (e) {
        case Event::BLE_CONNECTED:
          switch_states(SystemState::IDLE, SystemState::READY);
          haptic_notify(HapticEvent::BLE_CONNECT);
          break;

        default:
          break;
        }
        break;

      case SystemState::READY: // connected waiting for start
        // on run start, start collecting data
        switch (e) {
        case Event::TOGGLE_RUN:
          startSensors();
          switch_states(SystemState::READY, SystemState::RUNNING);
          haptic_notify(HapticEvent::RUN_START);
          break;

        case Event::BLE_DISCONNECTED:
          advertiseBLE();
          switch_states(SystemState::READY, SystemState::IDLE);
          haptic_notify(HapticEvent::BLE_DISCONNECT);
          break;

        default:
          break;
        }

        break;

      case SystemState::RUNNING: // logging and transmitting data
        switch (e) {
        case Event::TOGGLE_RUN:
          switch_states(SystemState::RUNNING, SystemState::READY);
          haptic_notify(HapticEvent::RUN_STOP);
          break;

        case Event::BLE_DISCONNECTED:
          advertiseBLE();
          switch_states(SystemState::RUNNING, SystemState::IDLE);
          haptic_notify(HapticEvent::BLE_DISCONNECT);
          break;

        default:
          break;
        }
        break;

      default: // could switch to default state being IDLE
        // Indicate Error
        xEventGroupSetBits(system_state, static_cast<EventBits_t>(SystemState::ERROR));
        logger.error("Error in state transition");
      }
    }
  }
}
// ---------------------------------------------------------------------
// ISRs
// ---------------------------------------------------------------------
// on boot button press
static void IRAM_ATTR boot_button_isr(void *arg) {
  Event e = Event::TOGGLE_RUN;
  // Start Run: if in idle -> running
  xQueueSendFromISR(eventQueue, &e, 0);
}

// ---------------------------------------------------------------------
//  BLE
// ---------------------------------------------------------------------

// callbacks to change system state on connection status
void on_ble_connect(NimBLEConnInfo &info) {
  Event e = Event::BLE_CONNECTED;
  BaseType_t status = xQueueSend(eventQueue, &e, 0);
  if (status != pdTRUE) {
    logger.warn("BLE connect event dropped");
  }

  logger.info("BLE: Client connected \n");
}

void on_ble_disconnect(NimBLEConnInfo &info, espp::BleGattServer::DisconnectReason reason) {

  Event e = Event::BLE_DISCONNECTED;
  BaseType_t status = xQueueSend(eventQueue, &e, 0);
  if (status != pdTRUE) {
    logger.warn("BLE disconnect event dropped");
  }

  logger.info("BLE: Client disconnected \n");
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

  vTaskDelay(pdMS_TO_TICKS(500)); // give i2c time to startup

  // init imu
  if (imu.init()) {
    logger.info("Imu initialized");
  } else {
    logger.error("Failed to initialize imu");
    return ESP_ERR_INVALID_STATE; // sometimes errors but works fine
  }

  // check if battery is connected
  if (!battery.isDeviceReady())
    logger.error("Could not initialize fuel gauge. Check battery connection");

  // setup force sensor
  pressure_sensor.set_calibration(FSR_LOW, FSR_HIGH);

  // Haptic setup
  if (!haptic.init()) {
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

  // init imu queue
  imuQueue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(quat_sample_t));

  if (imuQueue == nullptr) {
    ESP_LOGE("SYS_INIT", "Failed to create IMU queue.");
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
  ble.reconfigure(ble_config);
  ESP_RETURN_ON_ERROR(ble.init(), "SYS_INIT", "Failed to initialize BLE module");

  // Set MTU for larger packets
  ESP_RETURN_ON_FALSE(NimBLEDevice::setMTU(247), ESP_ERR_INVALID_STATE, "SYS_INIT",
                      "Failed to set MTU on NimBLEDevice.");

  logger.info("BLE module initialized successfully");

  // init event queue
  eventQueue = xQueueCreateStatic(EVENT_QUEUE_LENGTH, sizeof(Event), (uint8_t *)eventQueueStorage,
                                  &eventQueueStruct);
  if (eventQueue == nullptr) {
    ESP_LOGE("SYS_INIT", "Failed to create event queue");
    return ESP_ERR_INVALID_STATE;
  }

  // Create haptic queue
  hapticQueue = xQueueCreate(HAPTIC_QUEUE_LENGTH, sizeof(HapticEvent));
  if (hapticQueue == nullptr) {
    logger.error("Failed to create haptic queue");
  }

  // turn on led to indicate successful init
  red_led.turn_on();

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  Application Entry Point
// ---------------------------------------------------------------------

extern "C" void app_main() {
  // set log level for debug
  esp_log_level_set("FLASH_LOG", ESP_LOG_WARN);
  esp_log_level_set("BLE", ESP_LOG_WARN);

  // Setup
  if (initSystem() != ESP_OK) {
    logger.error("Failed initializing a system.");
  } // called once

  // Create Eventgroup
  system_state = xEventGroupCreate();

  // set initial state
  advertiseBLE();
  xEventGroupSetBits(system_state, static_cast<EventBits_t>(SystemState::IDLE));

  // Create Tasks
  // Create a task that will write samples to the flash
  xTaskCreate(flashWriterTask, "flash_writer", 4096, NULL, 6, &flash_writer_task_handle);

  // Create a task that will capture sensor data
  xTaskCreate(imuTask, "imu_task", 4096, NULL, 8, &imu_task_handle);

  // Create a task that will control program flow
  xTaskCreate(controlTask, "control", 4096, NULL, 9, &control_task_handle);

  // Create a task that will transmit stored data over BLE
  xTaskCreate(uploadTask, "upload", 4096, NULL, 4, &upload_task_handle);

  // Create a task that periodically updates the BLE battery level
  xTaskCreate(batteryTask, "battery", 3072, NULL, 2, &battery_task_handle);

  // Create a task that plays haptic events from the queue
  xTaskCreate(hapticTask, "haptic", 3072, NULL, 5, &haptic_task_handle);

  // Indicate boot complete
  haptic_notify(HapticEvent::BOOT);
}
