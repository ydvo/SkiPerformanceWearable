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
  SENSORS::Imu::Quaternion quat;
  int64_t timestamp_us;
};

// Capture sensor data and push to queue
void imuTask(void *arg) {
  TickType_t lastWake;
  quat_sample_t s;

  for (;;) {
    // Wait until Running
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    // get current time for scheduling
    lastWake = xTaskGetTickCount();

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {
      // save timestamp
      s.timestamp_us = esp_timer_get_time();

      // save imu data
      if (imu.update(IMU_FREQ * 1e-6)) {
        s.quat = imu.get_orientation();
      } else {
        s.quat = {1, 0, 0, 0}; // imu not ready, return identity quat
      }

      // push to queue (non-blocking, drop if full)
      if (xQueueSend(imuQueue, &s, 0) != pdTRUE) {
        logger.warn("IMU queue full, sample dropped");
      }

      // wait until imu frequency (100Hz)
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / IMU_FREQ));
    }
  }
}

// Batch read from queue and write to flash
void flashWriterTask(void *arg) {
  quat_sample_t batch[FLASH_BATCH_SIZE];
  size_t count;

  for (;;) {
    // Wait until Running
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    count = 0;

    // Block waiting for first sample
    if (xQueueReceive(imuQueue, &batch[count], portMAX_DELAY) == pdTRUE) {
      count++;

      // Non-blocking grab of remaining samples up to batch size
      while (count < FLASH_BATCH_SIZE && xQueueReceive(imuQueue, &batch[count], 0) == pdTRUE) {
        count++;
      }

      // Write batch to flash
      for (size_t i = 0; i < count; i++) {
        STORAGE::Quaternion q = {
            .w = batch[i].quat.w,
            .x = batch[i].quat.x,
            .y = batch[i].quat.y,
            .z = batch[i].quat.z,
        };
        if (flash_log.append(q, batch[i].timestamp_us) != ESP_OK) {
          logger.error("Failed to append sample to flash log");
        }
      }

      logger.debug("Wrote {} samples to flash", count);

      // Notify upload task that new data is available
      if (upload_task_handle != nullptr) {
        xTaskNotifyGive(upload_task_handle);
      }
    }
  }
}

// Read stored samples from flash and send over BLE
void uploadTask(void *arg) {
  STORAGE::FlashLog<STORAGE::Quaternion>::Frame buffered_frame;
  bool has_buffered_frame = false;
  size_t samples_sent_from_frame = 0;
  uint32_t total_frames_sent = 0;
  uint32_t total_ack_timeouts = 0;

  for (;;) {
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    // Reset buffered state when re-entering RUNNING
    has_buffered_frame = false;
    samples_sent_from_frame = 0;

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {

      // Buffer flash frame
      if (!has_buffered_frame) {
        size_t frames_read = 0;

        logger.debug("Upload: reading frame from flash");
        if (flash_log.read(&buffered_frame, 1, &frames_read) == ESP_OK && frames_read > 0) {
          has_buffered_frame = true;
          samples_sent_from_frame = 0;
          logger.info("Upload: loaded flash frame seq={} ({} samples per frame)",
                      (uint32_t)buffered_frame.seq, (uint32_t)STORAGE::SAMPLES_PER_FRAME);
        } else {
          logger.debug("Upload: no flash data – waiting for flash writer notification");
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          logger.debug("Upload: woke from flash writer notification");
          continue;
        }
      }

      // Wait for BLE connection
      if (!ble.is_connected()) {
        logger.debug("Upload: BLE not connected, retrying in {}ms", UPLOAD_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
        continue;
      }

      // Build BLE frame
      size_t samples_remaining = STORAGE::SAMPLES_PER_FRAME - samples_sent_from_frame;
      size_t samples_this_send =
          (samples_remaining > BLE::SAMPLES_PER_FRAME) ? BLE::SAMPLES_PER_FRAME : samples_remaining;

      // Calculate timestamp interpolation
      int64_t start_ts = buffered_frame.payload.start_t_us;
      int64_t end_ts = buffered_frame.payload.end_t_us;
      int64_t ts_step = (end_ts - start_ts) / (STORAGE::SAMPLES_PER_FRAME - 1);

      // Assemble frame
      BLE::Frame ble_frame = {};
      ble_frame.header.frame_seq = static_cast<uint16_t>(buffered_frame.seq);
      ble_frame.header.sample_count = static_cast<uint16_t>(samples_this_send);
      ble_frame.header.payload_len =
          static_cast<uint16_t>(samples_this_send * sizeof(BLE::FrameSample));
      ble_frame.header.flags = 0;

      for (size_t i = 0; i < samples_this_send; i++) {
        size_t src_idx = samples_sent_from_frame + i;
        ble_frame.payload[i].timestamp = static_cast<uint64_t>(start_ts + (ts_step * src_idx));
        ble_frame.payload[i].w = buffered_frame.payload.data[src_idx].w;
        ble_frame.payload[i].x = buffered_frame.payload.data[src_idx].x;
        ble_frame.payload[i].y = buffered_frame.payload.data[src_idx].y;
        ble_frame.payload[i].z = buffered_frame.payload.data[src_idx].z;
      }

      size_t payload_bytes = sizeof(BLE::FrameHeader) + ble_frame.header.payload_len;

      logger.info("Upload: sending frame seq={} samples={} offset={} payload_bytes={}",
                  (uint16_t)ble_frame.header.frame_seq, (uint16_t)samples_this_send,
                  (uint32_t)samples_sent_from_frame, (uint32_t)payload_bytes);

      // Send notification
      esp_err_t notify_err =
          ble.notify_quaternion(reinterpret_cast<const uint8_t *>(&ble_frame), payload_bytes);

      if (notify_err != ESP_OK) {
        logger.warn("Upload: notify_quaternion failed (err={}), retrying in {}ms", (int)notify_err,
                    UPLOAD_RETRY_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
        continue;
      }

      // Arm the ACK wait
      ble.arm_ack(ble_frame.header.frame_seq);

      logger.debug("Upload: notification sent, armed ACK wait for seq={}",
                   (uint16_t)ble_frame.header.frame_seq);

      // Poll for ACK with timeout
      int64_t wait_start = esp_timer_get_time();
      bool ack_received = false;
      int64_t last_poll_log_us = wait_start;

      while (esp_timer_get_time() - wait_start < UPLOAD_ACK_TIMEOUT_US) {
        if (ble.is_ack_received()) {
          ack_received = true;
          break;
        }

        // Periodic progress log every ~500ms
        int64_t now = esp_timer_get_time();
        if (now - last_poll_log_us >= 500'000) {
          logger.debug("Upload: still waiting for ACK seq={} ({:.1f}s elapsed)",
                       (uint16_t)ble_frame.header.frame_seq, (float)(now - wait_start) / 1e6f);
          last_poll_log_us = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
      }

      int64_t wait_us = esp_timer_get_time() - wait_start;

      if (!ack_received) {
        total_ack_timeouts++;
        logger.warn("Upload: ACK timeout after {:.2f}s for seq={} (total timeouts={})",
                    (float)wait_us / 1e6f, (uint16_t)ble_frame.header.frame_seq,
                    total_ack_timeouts);
        // Retry sending the same data next iteration (has_buffered_frame stays true)
        continue;
      }

      logger.info("Upload: ACK received for seq={} after {:.1f}ms",
                  (uint16_t)ble_frame.header.frame_seq, (float)wait_us / 1000.0f);

      // Advance send position
      samples_sent_from_frame += samples_this_send;

      logger.debug("Upload: frame seq={} progress {}/{} samples",
                   (uint16_t)ble_frame.header.frame_seq, (uint32_t)samples_sent_from_frame,
                   (uint32_t)STORAGE::SAMPLES_PER_FRAME);

      if (samples_sent_from_frame >= STORAGE::SAMPLES_PER_FRAME) {
        total_frames_sent++;
        has_buffered_frame = false;
        logger.info("Upload: flash frame seq={} fully transmitted (total frames sent={})",
                    (uint32_t)buffered_frame.seq, total_frames_sent);
      }
    }
  }
}

// Effect sequence table
namespace {
using E = HAPTICS::DRV2605L::EFFECTS;

struct HapticSequence {
  HapticEvent event;
  uint8_t effects[8]; // terminated by EFFECTS::END (0x00)
};

static const HapticSequence haptic_sequences[] = {
    // BOOT: soft double-click to confirm power-on
    {HapticEvent::BOOT, {E::DOUBLE_CLICK, E::END}},

    // BLE_CONNECT: rising two-pulse -- distinct from boot
    {HapticEvent::BLE_CONNECT, {E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK_2, E::END}},

    // BLE_DISCONNECT: single dull bump to signal loss
    {HapticEvent::BLE_DISCONNECT, {E::SOFT_BUMP, E::END}},

    // RUN_START: sharp strong click -- clear "go" signal
    {HapticEvent::RUN_START, {E::SHARP_CLICK, E::END}},

    // RUN_STOP: two soft clicks -- distinct from start
    {HapticEvent::RUN_STOP, {E::SOFT_BUMP, E::SOFT_BUMP_2, E::END}},

    // LOW_BATTERY: three short buzzes as a warning pattern
    {HapticEvent::LOW_BATTERY,
     {E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK, E::SHORT_DOUBLE_CLICK, E::END}},

    // ERROR: long buzz
    {HapticEvent::ERROR, {E::LONG_BUZZ, E::END}},
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
    uint8_t count = 0;
    while (count < 8 && seq->effects[count] != HAPTICS::DRV2605L::EFFECTS::END)
      count++;

    if (count > 0) {
      haptic.play(seq->effects, count);
      logger.debug("hapticTask: played event {}", static_cast<uint8_t>(event));
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

// TODO: interpret quadrature input

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
  xTaskCreate(flashWriterTask, "flash_writer", 4096, NULL, 3, &flash_writer_task_handle);

  // Create a task that will capture sensor data
  xTaskCreate(imuTask, "imu_task", 4096, NULL, 8, &imu_task_handle);

  // Create a task that will control program flow
  xTaskCreate(controlTask, "control", 4096, NULL, 9, &control_task_handle);

  // Create a task that will transmit stored data over BLE
  xTaskCreate(uploadTask, "upload", 4096, NULL, 4, &upload_task_handle);

  // Create a task that periodically updates the BLE battery level
  xTaskCreate(batteryTask, "battery", 2048, NULL, 2, &battery_task_handle);

  // Create a task that plays haptic events from the queue
  xTaskCreate(hapticTask, "haptic", 3072, NULL, 5, &haptic_task_handle);

  // Indicate boot complete
  haptic_notify(HapticEvent::BOOT);

  // to clear logs for debug
  esp_log_level_set("FLASH_LOG", ESP_LOG_WARN);
}
