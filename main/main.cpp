/** main.cpp - main application entry point and FreeRTOS task definitions */

#include "NimBLELocalValueAttribute.h"
#include "flash_log.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "imu.hpp"

#include "driver/spi_common.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
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

#include "esp_sleep.h"

#include <atomic>
#include <cstddef>
#include <cstdio>
#include <stdint.h>
#include <string>

// ---------------------------------------------------------------------
//  Hardware Pin Definitions
// ---------------------------------------------------------------------

/** @brief I2C port number used for communication. */
constexpr i2c_port_t i2c_port{I2C_NUM_0};

/** @brief GPIO number for I2C SDA line. */
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};

/** @brief GPIO number for I2C SCL line. */
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

/** @brief ADC channel for force-sensitive resistor pressure sensor. */
constexpr adc_channel_t fsr_pin{ADC_CHANNEL_7};

/** @brief GPIO number for SPI2 SCK (clock) line used for flash storage. */
static constexpr auto spi2_sck{GPIO_NUM_12};

/** @brief GPIO number for SPI2 MOSI (master out) line used for flash storage. */
static constexpr auto spi2_mosi{GPIO_NUM_11};

/** @brief GPIO number for SPI2 MISO (master in) line used for flash storage. */
static constexpr auto spi2_miso{GPIO_NUM_13};

/** @brief GPIO number for flash chip select (CS) line. */
static constexpr auto flash_spi_cs{GPIO_NUM_10};
// ---------------------------------------------------------------------
//  Global Objects
// ---------------------------------------------------------------------

// Logging
espp::Logger logger({.tag = "MAIN", .level = espp::Logger::Verbosity::DEBUG});

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

/** @brief Runtime flag indicating haptic hardware is present and initialized. */
static std::atomic<bool> has_haptics{false};

// Initialize LED
LED::led green_led = LED::led(LED::GREEN_LED);

// BLE Module
BLE::BleModule ble;

Common::GPIO big_button(GPIO_NUM_6, Common::GPIO::Direction::INPUT, Common::GPIO::Level::ON,
                        Common::GPIO::PULLUP);

// Stemma QT power control (I2C power rail)
Common::GPIO stemma_qt_power(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, Common::GPIO::Level::ON);

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------

/** @brief Polling interval for button state (ms). */
static constexpr uint32_t BUTTON_POLL_MS{20};

/** @brief Debounce time for button press (ms). */
static constexpr uint32_t BUTTON_DEBOUNCE_MS{50};

/** @brief Duration for detecting a long press (ms). */
static constexpr uint32_t LONG_PRESS_MS{2000};

/** @brief Duration button must be held after deep sleep wake to confirm boot (ms). */
static constexpr uint32_t WAKE_HOLD_MS{250};

/** @brief Minimum calibrated force sensor value (raw ADC units). */
static constexpr float FSR_LOW{0.00f};

/** @brief Maximum calibrated force sensor value (raw ADC units). */
static constexpr float FSR_HIGH{3000.0f};

/** @brief Polling interval for force sensor task (ms). */
static constexpr uint32_t FSR_POLL_INTERVAL_MS{50};

/** @brief RTP intensity value (0-255) for haptic feedback when threshold exceeded. */
static constexpr uint8_t FSR_HAPTIC_RTP_VALUE{127};

/** @brief Maximum duration (ms) for continuous haptic warning before auto-stop. */
static constexpr uint32_t FSR_HAPTIC_MAX_DURATION_MS{4000};

/** @brief Delay before calibration phase starts (ms). */
static constexpr uint32_t CAL_PHASE_DELAY_MS{3000};

/** @brief Threshold percentage of calibrated range to trigger events. */
static constexpr float FSR_THRESHOLD_PERCENT{0.01f};

/** @brief Threshold percentage of calibrated range to trigger events. */
static constexpr float FSR_THRESHOLD_MV{2999.0f};

/** @brief Queue size for IMU samples. */
static constexpr uint8_t IMU_QUEUE_SIZE{64};

/** @brief Number of samples to batch for flash writes. */
static constexpr uint8_t FLASH_BATCH_SIZE{14};

/** @brief Number of samples to send over BLE. */
static constexpr uint8_t NUM_SAMPlES_BLE{24};

/** @brief Retry delay for upload task after failure (ms). */
static constexpr uint32_t UPLOAD_RETRY_DELAY_MS{100};

/** @brief Timeout for upload acknowledgment (microseconds). */
static constexpr int64_t UPLOAD_ACK_TIMEOUT_US{5'000'000};

/** @brief Interval for battery status updates (ms). */
static constexpr uint32_t BATTERY_UPDATE_INTERVAL_MS{30'000};

/** @brief IMU sampling frequency (Hz). */
static constexpr float IMU_FREQ{100};

/** @brief Length of event queue (number of entries). */
static constexpr uint8_t EVENT_QUEUE_LENGTH{8};

/** @brief Length of haptic event queue (number of entries). */
static constexpr uint8_t HAPTIC_QUEUE_LENGTH{4};

/** @brief Number of samples to take when calibrating force sensor threshuld */
static constexpr float NUM_SAMPLES_CALIBRATION{10.0};

/*! @brief Haptic event identifiers for the DRV2605L driver.
 *
 * Each value corresponds to a distinct haptic feedback pattern defined in
 * `haptic_sequences`.
 */
enum class HapticEvent : uint8_t {
  BOOT,           // device powered on and init succeeded
  BLE_CONNECT,    // client connected
  BLE_DISCONNECT, // client disconnected
  RUN_START,      // recording started
  RUN_STOP,       // recording stopped
  LOW_BATTERY,    // battery below threshold
  ERROR,          // unrecoverable error
  CAL_START,      // calibration started (release pressure)
  CAL_PHASE_TWO,  // apply full pressure now
  CAL_COMPLETE,   // calibration finished
  SLEEP,          // device entering deep sleep
};

// ---------------------------------------------------------------------
// Setup for Control Flow
// ---------------------------------------------------------------------
// possible states
/*! @brief System state bitmask used with FreeRTOS EventGroup.
 *
 * Each state occupies a distinct bit; multiple bits may be set simultaneously.
 */
enum class SystemState : EventBits_t {
  SLEEP = BIT0,
  IDLE = BIT1,
  READY = BIT2,
  RUNNING = BIT3,
  ERROR = BIT4,
  CALIBRATING = BIT5,
};

// state var
static EventGroupHandle_t system_state;

// events
/*! @brief Application events posted to the control task queue. */
enum class Event : uint8_t {
  BLE_CONNECTED,
  BLE_DISCONNECTED,
  TOGGLE_RUN,
  START_CALIBRATION,
  CALIBRATION_DONE,
  SLEEP_REQUEST,
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
static TaskHandle_t fsr_task_handle = nullptr;
static TaskHandle_t calibration_task_handle = nullptr;
static TaskHandle_t led_task_handle = nullptr;
static TaskHandle_t button_task_handle = nullptr;

// queue for imu data
static QueueHandle_t imuQueue = nullptr;

// queue for haptic events
static QueueHandle_t hapticQueue = nullptr;

/**
 * @brief Enqueue a haptic event for asynchronous processing.
 *
 * The function posts the specified @p event to the global @c hapticQueue.
 * If the queue is not yet created or is full, the event is dropped silently.
 * This call is non‑blocking and safe to invoke from any FreeRTOS task
 * (including ISRs, as it uses the non‑blocking queue API).
 *
 * @param event Identifier of the haptic event to enqueue.
 */
static inline void haptic_notify(HapticEvent event) {
  if (!has_haptics.load(std::memory_order_relaxed))
    return;
  if (hapticQueue != nullptr) {
    xQueueSend(hapticQueue, &event, 0); // non-blocking, drop if queue full
  }
}

// structure of data to be stored in sensor data queue
/*! @brief Queued IMU sample containing timestamp and quaternion. */
struct quat_sample_t {
  uint64_t timestamp_us;
  SENSORS::Imu::Quaternion quat;
};

// Capture sensor data and push to queue
/*!
 * @brief FreeRTOS task that reads IMU samples at IMU_FREQ and enqueues them.
 *
 * The task blocks on the RUNNING state event group, timestamps each sample
 * with `esp_timer_get_time()`, and attempts a non‑blocking enqueue to
 * `imuQueue`. If the queue is full the sample is dropped and a warning is logged.
 *
 * @param arg Unused task argument (must be nullptr when created).
 */
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
/*!
 * @brief FreeRTOS task that dequeues IMU samples and writes them to flash.
 *
 * While the system is in RUNNING state it blocks on `imuQueue` for the first
 * sample, then appends each received quaternion to the flash log. Errors are
 * logged via `logger.error`.
 *
 * @param arg Unused task argument.
 */
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

/*! @brief State machine for flash-to-BLE upload handling. */
enum class UploadState : uint8_t {
  BUFFER_FLASH_FRAME,
  INITIALIZE_BLE_FRAME,
  NOTIFY_BLE,
  WAIT_FOR_ACK,
  RECEIVED_ACK,
  TIMEOUT_ACK,
};

// Read stored samples from flash and send over BLE
/*!
 * @brief FreeRTOS task that streams flash‑stored IMU data over BLE.
 *
 * The task reads frames from `flash_log`, builds BLE frames and notifies the
 * client. It then waits for an acknowledgment with a timeout defined by
 * `UPLOAD_ACK_TIMEOUT_US`. Retransmission and timeout handling are performed
 * according to `UploadState`.
 *
 * @param arg Unused task argument.
 */
void uploadTask(void *arg) {
  uint32_t total_frames_sent = 0;
  uint32_t total_ack_timeouts = 0;
  static STORAGE::FlashLog<STORAGE::Quaternion>::Frame flash_frame;
  static BLE::Frame ble_frame{
      .header = {.frame_seq = 0, .sample_count = 0, .payload_len = 0, .flags = 0}, .payload = {}};

  for (;;) {
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    UploadState upload_state = UploadState::BUFFER_FLASH_FRAME;

    size_t samples_sent_from_frame{0};
    int64_t ble_ack_start;
    int64_t ack_wait_us;

    // Per-run drain tracking: set once a flash frame is successfully sent,
    // used to detect when upload is complete after recording stops.
    bool ever_had_data = false;
    bool drain_notified = false;

    while (true) {
      EventBits_t bits = xEventGroupGetBits(system_state);
      bool still_running = bits & static_cast<EventBits_t>(SystemState::RUNNING);
      bool in_ready = bits & static_cast<EventBits_t>(SystemState::READY);

      // Exit once drain notification has been sent (or there was nothing to drain)
      if (drain_notified)
        break;
      // Exit if the system left RUNNING/READY (e.g. BLE disconnected → IDLE)
      if (!still_running && !in_ready)
        break;

      switch (upload_state) {
      case UploadState::BUFFER_FLASH_FRAME: {
        size_t frames_read{0};

        logger.debug("Upload: reading frame from flash");
        if (flash_log.read(&flash_frame, 1, &frames_read) == ESP_OK && frames_read > 0) {
          ever_had_data = true;
          samples_sent_from_frame = 0;
          logger.info("Upload: loaded flash frame seq={} ({} samples per frame)",
                      (uint32_t)flash_frame.seq, (uint32_t)STORAGE::SAMPLES_PER_FRAME);

          upload_state = UploadState::INITIALIZE_BLE_FRAME;
        } else {
          if (!still_running) {
            // Recording has stopped and flash is empty – drain complete
            if (ever_had_data) {
              if (ble.notify_upload_complete() == ESP_OK) {
                logger.info("Upload: flash fully drained, central notified");
              } else {
                logger.warn("Upload: flash drained but notify_upload_complete failed");
              }
            } else {
              logger.info("Upload: run ended with no complete flash frames to drain");
            }
            drain_notified = true;
          } else {
            logger.debug("Upload: no flash data, retrying in {}ms", UPLOAD_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(UPLOAD_RETRY_DELAY_MS));
          }
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

        logger.debug("Upload: sending frame seq={} samples={} offset={} payload_bytes={}",
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
          // Only ACK the last sub-frame of each flash frame to halve round-trips.
          // Non-last sub-frames are sent fire-and-forget; the app ACKs only the
          // last one, which also implicitly confirms all preceding sub-frames.
          size_t next_samples = samples_sent_from_frame + ble_frame.header.sample_count;
          bool is_last_subframe = (next_samples >= STORAGE::SAMPLES_PER_FRAME);

          if (is_last_subframe) {
            ble.arm_ack(ble_frame.header.frame_seq);
            logger.debug("Upload: notification sent, armed ACK wait for seq={}",
                         (uint16_t)ble_frame.header.frame_seq);
            upload_state = UploadState::WAIT_FOR_ACK;
            ble_ack_start = esp_timer_get_time();
          } else {
            // Advance and send next sub-frame immediately without waiting for ACK.
            samples_sent_from_frame = next_samples;
            upload_state = UploadState::INITIALIZE_BLE_FRAME;
          }
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
        // RECEIVED_ACK is only reached for the last sub-frame of a flash frame.
        logger.info("Upload: ACK received for seq={} after {:.1f}ms",
                    (uint16_t)ble_frame.header.frame_seq, (float)ack_wait_us / 1000.0f);

        samples_sent_from_frame += ble_frame.header.sample_count;
        total_frames_sent++;
        logger.info("Upload: flash frame seq={} fully transmitted (total frames sent={})",
                    (uint32_t)flash_frame.seq, total_frames_sent);

        upload_state = UploadState::BUFFER_FLASH_FRAME;
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

/**
 * @brief Mapping from HapticEvent to DRV2605L effect sequence.
 *
 * Each entry defines the number of effects (`count`) and an array of effect
 * identifiers terminated by `EFFECTS::END`. This table is used by the hapticTask
 * to look up the appropriate pattern for a given @c HapticEvent.
 */
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

    // CAL_START: ramp up to signal calibration started (release pressure)
    {HapticEvent::CAL_START, 1, {E::RAMP_UP, E::END}},

    // CAL_PHASE_TWO: double click to signal "apply pressure now"
    {HapticEvent::CAL_PHASE_TWO, 2, {E::DOUBLE_CLICK, E::DOUBLE_CLICK, E::END}},

    // CAL_COMPLETE: ramp down to signal calibration finished
    {HapticEvent::CAL_COMPLETE, 1, {E::RAMP_DOWN, E::END}},

    // SLEEP: descending pattern to signal device going to sleep
    {HapticEvent::SLEEP, 3, {E::RAMP_DOWN, E::TRANSITION_HUM, E::SOFT_BUMP, E::END}},
};
} // namespace

// Dequeue HapticEvents and play the corresponding effect sequence
/*!
 * @brief FreeRTOS task that processes queued HapticEvent values.
 *
 * The task blocks on `hapticQueue`, looks up the corresponding effect
 * sequence in `haptic_sequences`, and plays it via the DRV2605L driver. If an
 * unknown event is received a warning is logged.
 *
 * @param arg Unused task argument.
 */
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

// Monitor FSR pressure and drive haptic motor in realtime when threshold exceeded
/*!
 * @brief FreeRTOS task that monitors the force‑sensitive resistor.
 *
 * While the system is RUNNING the task reads the sensor voltage and checks
 * whether the pressure exceeds the calibrated threshold. If the threshold is
 * crossed a realtime haptic warning is started; when pressure falls back below
 * the threshold the warning is stopped.
 *
 * @param arg Unused task argument.
 */
void fsrTask(void *arg) {
  bool warning_active = false;
  bool warning_timed_out = false;
  TickType_t warning_start_tick{0};

  for (;;) {
    // Block until RUNNING state
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::RUNNING), false, true,
                        portMAX_DELAY);

    while (xEventGroupGetBits(system_state) & static_cast<EventBits_t>(SystemState::RUNNING)) {
      float voltage = pressure_sensor.read();
      // printf("%f\n", voltage);
      bool exceeded = pressure_sensor.is_pressed();

      if (exceeded && !warning_active && !warning_timed_out) {
        // Threshold crossed -- switch to realtime playback and buzz
        logger.info("FSR: threshold exceeded ({:.1f} mV), activating warning", voltage);
        haptic.set_mode(HAPTICS::DRV2605L::Mode::REALTIME_PLAYBACK);
        haptic.set_realtime_value(FSR_HAPTIC_RTP_VALUE);
        warning_active = true;
        warning_start_tick = xTaskGetTickCount();

      } else if (!exceeded && (warning_active || warning_timed_out)) {
        // Pressure dropped below threshold -- stop buzzing and restore mode
        if (warning_active) {
          logger.info("FSR: pressure released ({:.1f} mV), deactivating warning", voltage);
          haptic.set_realtime_value(0);
          haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);
        }
        warning_active = false;
        warning_timed_out = false;
      }

      // Auto-stop warning after maximum duration to avoid prolonged buzzing
      if (warning_active &&
          (xTaskGetTickCount() - warning_start_tick) >= pdMS_TO_TICKS(FSR_HAPTIC_MAX_DURATION_MS)) {
        logger.info("FSR: warning timed out after {}ms, deactivating", FSR_HAPTIC_MAX_DURATION_MS);
        haptic.set_realtime_value(0);
        haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);
        warning_active = false;
        warning_timed_out = true;
      }

      vTaskDelay(pdMS_TO_TICKS(FSR_POLL_INTERVAL_MS));
    }

    // Exiting RUNNING state -- ensure motor is off and mode restored
    if (warning_active) {
      haptic.set_realtime_value(0);
      haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);
      warning_active = false;
    }
    warning_timed_out = false;
  }
}

// Run FSR calibration sequence guided by haptic feedback
/*!
 * @brief FreeRTOS task that guides the user through FSR calibration.
 *
 * The task runs when the system enters the CALIBRATING state. It uses haptic
 * feedback to prompt the user to assume minimum leang angle they want to be notified at.
 * It proceeds to take NUM_SAMPLES_CALIBRATION samples and average them to set the fsr threshold.
 * Upon completion it notifies the control task.
 *
 * @param arg Unused task argument.
 */
void calibrationTask(void *arg) {
  for (;;) {
    // Block until CALIBRATING state
    xEventGroupWaitBits(system_state, static_cast<EventBits_t>(SystemState::CALIBRATING), false,
                        true, portMAX_DELAY);

    // Notify and start reading values
    haptic_notify(HapticEvent::CAL_START);
    logger.info("Calibration: Lean to minimum angle");
    vTaskDelay(pdMS_TO_TICKS(CAL_PHASE_DELAY_MS));
    float sum = 0;
    for (uint8_t i = 0; i < NUM_SAMPLES_CALIBRATION; i++) {
      sum += pressure_sensor.read();
    }
    float threshold = sum / NUM_SAMPLES_CALIBRATION;

    pressure_sensor.set_pressure_threshold(threshold);
    logger.info("Calibration: threshold set to {:.1f} mV", threshold);

    // Signal completion
    haptic_notify(HapticEvent::CAL_COMPLETE);

    Event e = Event::CALIBRATION_DONE;
    xQueueSend(eventQueue, &e, 0);
  }
}

// ----- Button ISR and Task -----

/*!
 * @brief GPIO ISR for the button (falling edge).
 *
 * Wakes the button task via a direct-to-task notification. Debounce and
 * long-press detection are handled in the task, not here.
 */
static void IRAM_ATTR button_isr(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*!
 * @brief FreeRTOS task that debounces button presses and detects long-press.
 *
 * Long press (>=3s): posts SLEEP_REQUEST event.
 *
 * @param arg Unused task argument.
 */
void buttonTask(void *arg) {
  for (;;) {
    // Block until ISR fires (falling edge = button pressed)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Debounce: wait and re-check level
    vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    if (big_button.getLevel()) {
      continue; // false trigger, button already released
    }

    // Button is held. Time the hold duration.
    uint32_t held_ms = BUTTON_DEBOUNCE_MS;
    bool long_press = false;
    while (!big_button.getLevel()) {
      vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
      held_ms += BUTTON_POLL_MS;
      if (held_ms >= LONG_PRESS_MS) {
        long_press = true;
        break;
      }
    }

    if (long_press) {
      Event e = Event::SLEEP_REQUEST;
      xQueueSend(eventQueue, &e, 0);
    }
    // Short press is ignored -- runs are started/stopped over BLE only
  }
}

// Periodically read fuel gauge and push battery level to BLE battery service
/*!
 * @brief FreeRTOS task that periodically reads the MAX1704X fuel gauge.
 *
 * When BLE is connected the task publishes the battery level (percentage)
 * to the BLE Battery Service. Errors updating the characteristic are logged.
 * The task runs at `BATTERY_UPDATE_INTERVAL_MS` intervals.
 *
 * @param arg Unused task argument.
 */
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

uint8_t LED_LOW_FREQ{1};   // 1 Hz frequency -> period is 2 seconds
uint8_t LED_MID_FREQ{2};   // 2 Hz frequency -> period is 1 seconds
uint8_t LED_HIGH_FREQ{10}; // 10 Hz frequency -> period is 0.2 seconds

void ledTask(void *arg) {
  static SystemState currstate;

  for (;;) {
    currstate = static_cast<SystemState>(xEventGroupGetBits(system_state));

    switch (currstate) {
    case SystemState::SLEEP:
      // Turn off led when device is asleep
      green_led.turn_off();
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;
    case SystemState::IDLE:
      // Not connected, flash periodically with medium frequency
      green_led.toggle();
      vTaskDelay(pdMS_TO_TICKS(1000 / LED_MID_FREQ));
      break;
    case SystemState::READY:
      // Connected, led stays on
      green_led.turn_on();
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;
    case SystemState::RUNNING:
      // Session running, flash periodically with low frequency
      green_led.toggle();
      vTaskDelay(pdMS_TO_TICKS(1000 / LED_LOW_FREQ));
      break;
    case SystemState::ERROR:
      // Error state, turn off led
      green_led.turn_off();
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;
    case SystemState::CALIBRATING:
      // Device calibrating, flash periodically with high frequency
      green_led.toggle();
      vTaskDelay(pdMS_TO_TICKS(1000 / LED_HIGH_FREQ));
      break;
    default:
      vTaskDelay(pdMS_TO_TICKS(1000));
      break;
    }
  }
}
// ---------------------------------------------------------------------
// State Machine
// ---------------------------------------------------------------------

// idle mode callback to sleep everything and start ble_advertising
/*!
 * @brief Starts BLE advertising and logs the device name.
 *
 * Intended to be called when the system enters the SLEEP or IDLE state.
 * TODO: add power‑down of IMU, FSR, and haptics.
 *
 * @return ESP_OK on success or an ESP‑IDF error code propagated from
 *         `ble.start_advertising()`.
 */
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

/*!
 * @brief Placeholder for waking up sensors (IMU, FSR) and starting haptics.
 *
 * Currently a stub; future implementation should power up peripherals and
 * configure them for data collection.
 *
 * @return ESP_OK on success (always returns ESP_OK in current stub).
 */
esp_err_t startSensors() {
  /* TODO:
   *  - wake imu
   *  - wake fsr
   *  - start haptics
   */

  return ESP_OK;
}

// helper to transition states
/*!
 * @brief Helper that atomically updates the system state event group.
 *
 * Sets bits for `nextstate` and clears bits for `currstate`. Also logs the
 * transition using `logger.info`.
 *
 * @param currstate Current system state (bits to clear).
 * @param nextstate Target system state (bits to set).
 */
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

  case SystemState::CALIBRATING:
    s = "CALIBRATING";
    break;
  default:
    s = "ERROR";
    break;
  }
  logger.info("State: {}", s);
}

// ----- Deep Sleep -----

/*!
 * @brief Shuts down peripherals and enters ESP32-S3 deep sleep.
 *
 * Sequence: haptic buzz, turn off LED, deinit BLE, sleep IMU,
 * standby haptic driver, disable I2C power rail, configure ext1 wakeup
 * on GPIO_NUM_6 (button), then enter deep sleep. Does not return --
 * the chip reboots through app_main() on wake.
 */
static void enterDeepSleep() {
  logger.info("Entering deep sleep...");

  // 1. Haptic buzz
  haptic_notify(HapticEvent::SLEEP);
  vTaskDelay(pdMS_TO_TICKS(500));

  // 2. Turn off LED
  green_led.turn_off();

  // 3. Deinit BLE (stop advertising, disconnect, tear down NimBLE stack)
  ble.deinit();

  // 4. Put IMU into hardware sleep
  imu.sleep();

  // 5. Put haptic driver into standby
  if (has_haptics.load(std::memory_order_relaxed)) {
    haptic.enter_standby();
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  // 6. Disable Stemma QT power (kills I2C power rail)
  stemma_qt_power.setLevel(false);

  // 7. Configure ext1 wakeup on GPIO_NUM_6 (active low with pull-up)
  esp_sleep_enable_ext1_wakeup(1ULL << GPIO_NUM_6, ESP_EXT1_WAKEUP_ANY_LOW);

  // 8. Enter deep sleep (does not return)
  esp_deep_sleep_start();
}

// state machine
//  - transitions on Events
/*!
 * @brief FreeRTOS task implementing the system state machine.
 *
 * Waits for `Event` messages on `eventQueue` and transitions between
 * `SystemState` values, invoking appropriate actions such as BLE advertising,
 * haptic notifications, and sensor control.
 *
 * @param arg Unused task argument.
 */
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

        case Event::SLEEP_REQUEST:
          enterDeepSleep(); // does not return
          break;

        default:
          break;
        }
        break;

      case SystemState::READY: // connected waiting for start
        // on run start, start collecting data
        switch (e) {
        case Event::TOGGLE_RUN:
          flash_log.init(); // reset pointers so each run starts with an empty log
          startSensors();
          switch_states(SystemState::READY, SystemState::RUNNING);
          haptic_notify(HapticEvent::RUN_START);
          break;

        case Event::START_CALIBRATION:
          switch_states(SystemState::READY, SystemState::CALIBRATING);
          break;

        case Event::BLE_DISCONNECTED:
          advertiseBLE();
          switch_states(SystemState::READY, SystemState::IDLE);
          haptic_notify(HapticEvent::BLE_DISCONNECT);
          break;

        case Event::SLEEP_REQUEST:
          enterDeepSleep(); // does not return
          break;

        default:
          break;
        }

        break;

      case SystemState::CALIBRATING: // FSR calibration in progress
        switch (e) {
        case Event::CALIBRATION_DONE:
          switch_states(SystemState::CALIBRATING, SystemState::READY);
          break;

        case Event::BLE_DISCONNECTED:
          advertiseBLE();
          switch_states(SystemState::CALIBRATING, SystemState::IDLE);
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
          // Keep recording — re-advertise so the phone can reconnect.
          // The upload task will pause automatically until BLE is back.
          advertiseBLE();
          haptic_notify(HapticEvent::BLE_DISCONNECT);
          break;

        case Event::BLE_CONNECTED:
          haptic_notify(HapticEvent::BLE_CONNECT);
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
//  BLE
// ---------------------------------------------------------------------

// callbacks to change system state on connection status
/*!
 * @brief BLE connection callback.
 *
 * Enqueues `Event::BLE_CONNECTED` to the control task's event queue. Logs a
 * warning if the queue is full. Also emits an informational log.
 *
 * @param info Connection info (unused).
 */
void on_ble_connect(NimBLEConnInfo &info) {
  Event e = Event::BLE_CONNECTED;
  BaseType_t status = xQueueSend(eventQueue, &e, 0);
  if (status != pdTRUE) {
    logger.warn("BLE connect event dropped");
  }

  logger.info("BLE: Client connected \n");
}

/*!
 * @brief BLE disconnection callback.
 *
 * Enqueues `Event::BLE_DISCONNECTED` to the control task's event queue. Logs a
 * warning if the queue is full and an informational message.
 *
 * @param info Connection info (unused).
 * @param reason Reason for disconnection (unused).
 */
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

/*!
 * @brief Performs application-wide initialization of peripherals and components.
 *
 * Sets up I2C, IMU, fuel gauge, force sensor, haptic driver, SPI flash, BLE,
 * and queues. Logs progress and returns an ESP‑IDF error code on failure.
 *
 * @return ESP_OK on success or an error code indicating the failure point.
 */
esp_err_t initSystem() {
  logger.info("Initializing...");

  // enable QT Stemma Port (global stemma_qt_power already constructed with ON level)
  logger.info("Enabled QT Stemma Port");

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
    // return ESP_ERR_INVALID_STATE; // sometimes errors but works fine
  }

  // check if battery is connected
  if (!battery.isDeviceReady())
    logger.error("Could not initialize fuel gauge. Check battery connection");

  // setup force sensor
  pressure_sensor.set_calibration(FSR_LOW, FSR_HIGH);

  // set pressure threshold at 50% of calibrated range
  float fsr_threshold = FSR_THRESHOLD_MV;
  pressure_sensor.set_pressure_threshold(fsr_threshold);
  logger.info("FSR threshold set to {:.1f} mV", fsr_threshold);

  // Haptic setup (optional hardware -- non-fatal if absent)
  has_haptics.store(haptic.init(), std::memory_order_relaxed);
  if (!has_haptics.load(std::memory_order_relaxed)) {
    logger.warn("Haptic motor not detected -- haptic features disabled");
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

  // BLE control characteristic callback – 0x01 toggles recording, 0x02 toggles calibration
  ble_config.on_control_command = [](uint8_t cmd) {
    Event e;
    BaseType_t status;

    switch (cmd) {
    case 0x01:
      e = Event::TOGGLE_RUN;
      status = xQueueSend(eventQueue, &e, 0);
      if (status != pdTRUE) {
        logger.warn("BLE control: TOGGLE_RUN event dropped (queue full)");
      } else {
        logger.info("BLE control: TOGGLE_RUN enqueued");
      }
      break;
    case 0x02:
      e = Event::START_CALIBRATION;
      status = xQueueSend(eventQueue, &e, 0);
      if (status != pdTRUE) {
        logger.warn("BLE control: START_CALIBRATION event dropped (queue full)");
      } else {
        logger.info("BLE control: START_CALIBRATION enqueued");
      }
      break;
    default:
      logger.warn("BLE control: unknown command 0x{:02x}", (unsigned)cmd);
    }
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

  // Create haptic queue (only if haptic hardware is present)
  if (has_haptics.load(std::memory_order_relaxed)) {
    hapticQueue = xQueueCreate(HAPTIC_QUEUE_LENGTH, sizeof(HapticEvent));
    if (hapticQueue == nullptr) {
      logger.error("Failed to create haptic queue");
    }
  }

  // turn on led to indicate successful init
  green_led.turn_on();

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  Application Entry Point
// ---------------------------------------------------------------------

/*!
 * @brief Application entry point required by ESP-IDF.
 *
 * Sets log levels, calls `initSystem()`, creates the system state event
 * group, starts BLE advertising, creates all FreeRTOS tasks, and notifies the
 * haptic motor that boot is complete.
 */
extern "C" void app_main() {
  // Detect wakeup cause and validate sustained button hold
  esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
  if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT1) {
    ESP_LOGI("MAIN", "Woke from deep sleep -- validating button hold");

    uint32_t held_ms = 0;
    bool confirmed = true;
    while (held_ms < WAKE_HOLD_MS) {
      if (big_button.getLevel()) {
        // Button released too early -- go back to sleep
        confirmed = false;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
      held_ms += BUTTON_POLL_MS;
    }

    if (!confirmed) {
      ESP_LOGI("MAIN", "Button released early -- returning to deep sleep");
      esp_sleep_enable_ext1_wakeup(1ULL << GPIO_NUM_6, ESP_EXT1_WAKEUP_ANY_LOW);
      esp_deep_sleep_start();
      // does not return
    }

    ESP_LOGI("MAIN", "Button hold confirmed -- booting");
  } else {
    ESP_LOGI("MAIN", "Cold boot (power-on or reset)");
  }

  // set log level for debug
  esp_log_level_set("FLASH_LOG", ESP_LOG_WARN);
  esp_log_level_set("BLE", ESP_LOG_WARN);
  esp_log_level_set("LED", ESP_LOG_WARN);

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

  // Create a task that manages led state
  xTaskCreate(ledTask, "led_task", 2048, NULL, 3, &led_task_handle);

  // Create a task that will control program flow
  xTaskCreate(controlTask, "control", 4096, NULL, 9, &control_task_handle);

  // Create a task that handles button press debounce and long-press detection
  xTaskCreate(buttonTask, "button", 2048, NULL, 7, &button_task_handle);

  // Register button ISR after task handle is valid
  if (big_button.set_interrupt(Common::GPIO::INTERRUPT_FALLING_EDGE, button_isr, nullptr) !=
      ESP_OK) {
    logger.error("Failed to set button interrupt");
  }

  // Create a task that will transmit stored data over BLE
  xTaskCreate(uploadTask, "upload", 4096, NULL, 4, &upload_task_handle);

  // Create a task that periodically updates the BLE battery level
  xTaskCreate(batteryTask, "battery", 3072, NULL, 2, &battery_task_handle);

  // Create a task that plays haptic events from the queue (only if hardware present)
  if (has_haptics.load(std::memory_order_relaxed)) {
    xTaskCreate(hapticTask, "haptic", 3072, NULL, 5, &haptic_task_handle);

    // Create a task that monitors FSR and triggers haptic warning
    xTaskCreate(fsrTask, "fsr_task", 4096, NULL, 5, &fsr_task_handle);
  }

  // Create a task that runs the FSR calibration sequence
  xTaskCreate(calibrationTask, "calibrate", 4096, NULL, 5, &calibration_task_handle);

  // Indicate boot complete
  haptic_notify(HapticEvent::BOOT);
}
