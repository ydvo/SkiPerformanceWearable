#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_attr.h"
#include "esp_timer.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "flash.hpp"
#include "flash_log.hpp"
#include "ble.hpp"

#include <cstdio>
#include <stdint.h>
#include <array>
#include <vector>

using namespace std::chrono_literals;

static esp_timer_handle_t sensor_timer; 
constexpr uint64_t sensor_polling_period{10000}; // 10 ms = 10_000 us

// ---------------------------------------------------------------------
//  Hardware Pin Definitions
// ---------------------------------------------------------------------

// ICM-20948 IMU
constexpr uint8_t ICM20948_ADDRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};  // 400 kHz

// I2C pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// SPI2 pins (for flash storage)
static constexpr auto spi2_sck{GPIO_NUM_12};
static constexpr auto spi2_mosi{GPIO_NUM_11};
static constexpr auto spi2_miso{GPIO_NUM_13};
static constexpr auto flash_spi_cs{GPIO_NUM_10};

// ---------------------------------------------------------------------
//  Global Objects
// ---------------------------------------------------------------------

typedef enum {
  system_state_idle, 
  system_state_recording, 
  system_state_complete,
  system_state_flushing, 
  system_state_uploading, 
} system_state_t; 

static system_state_t system_state; 

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
STORAGE::SpiFlashDevice spi_flash({
  .host = SPI2_HOST,
  .cs = flash_spi_cs
});


// Flash Log
STORAGE::FlashLog<STORAGE::Quaternion> flash_log(
  spi_flash
);
// IMU
SENSORS::Imu imu(i2c);

// BLE Module
BLE::BleModule *ble_module_ptr = nullptr;

// State Variables
float dt = 0;
uint8_t battery_level = 100;

/**
 * Flushes the flash contents over UART
 */
void flush_flash_to_host() {
  STORAGE::FlashLog<STORAGE::Quaternion>::Frame frames[8]; 
  size_t frames_read;
  uint32_t read_addr = flash_log.read_addr(); 
  uint32_t start_addr = read_addr; 

  printf("START\n");

  for (;;) {
    if (flash_log.read_immut(frames, 8, &frames_read, read_addr, &read_addr) != ESP_OK) {
      printf("ERROR\n"); 
      break; 
    }

    if (frames_read == 0) {
      printf("END\n"); 
      printf("Completed reading range 0x%x - 0x%x\n", (unsigned int) start_addr, (unsigned int) (read_addr - 1)); 
      break;
    }

    for (size_t i = 0; i < frames_read; ++i) {
      auto &frame = frames[i];

      printf("%lld", frame.payload.start_t_us); 
      for (int j = 0; j < STORAGE::SAMPLES_PER_FRAME; ++j) {
        printf(",%.6f,%.6f,%.6f,%.6f",frame.payload.data[j].w, frame.payload.data[j].x, frame.payload.data[j].y, frame.payload.data[j].z);
      }
      printf(",%lld\n", frame.payload.end_t_us);
      // allow scheduler to run
      vTaskDelay(1);
    }
  }
}

static TaskHandle_t flash_writer_task_handle = nullptr; 
static TaskHandle_t capture_task_handle = nullptr;
static TaskHandle_t upload_task_handle = nullptr; 
static TaskHandle_t control_task_handle = nullptr;

constexpr uint32_t NOTIFY_BUTTON_PRESS = (1 << 0); 
constexpr uint32_t NOTIFY_FLASH_DONE = (1 << 1); 
constexpr uint32_t NOTIFY_UPLOAD_DONE = (1 << 2); 
constexpr uint32_t NOTIFY_UPLOAD_UART = (1 << 3); 
constexpr uint32_t NOTIFY_UPLOAD_BLE = (1 << 4); 


/**
 * Flash Writer Task
 */
RingbufHandle_t flash_writer_buf = nullptr; 
struct quat_sample_t{
  STORAGE::Quaternion quat; 
  int64_t timestamp_us;
};
void flash_writer_task(void *arg) {
  quat_sample_t *item; 
  size_t size; 

  for (;;) {
    item = (quat_sample_t *)xRingbufferReceive(
      flash_writer_buf, 
      &size, 
      pdMS_TO_TICKS(50) // timeout
    );

    if (item) {
      if (size != 0 && flash_log.append(item->quat, item->timestamp_us) != ESP_OK) {
        ESP_LOGE("FLASH_WRITER", "Failed to append quat into flash log."); 
      }

      vRingbufferReturnItem(flash_writer_buf, item); 
    }

    if (system_state == system_state_flushing && 
      xRingbufferGetCurFreeSize(flash_writer_buf) == xRingbufferGetMaxItemSize(flash_writer_buf)
    ) {
      xTaskNotify(
        control_task_handle, 
        NOTIFY_FLASH_DONE, 
        eSetBits
      ); 
    }
  }
}

/**
 * Upload Task
 *  - performs data transmission over BLE or flashes over UART
 *  - can use this to initiate ble transmission
 */

void upload_task(void *arg) {
  uint32_t notify_val; 

  for (;;) {
    xTaskNotifyWait(
      0, 
      UINT32_MAX, 
      &notify_val, 
      portMAX_DELAY
    ); 

    if (notify_val & NOTIFY_UPLOAD_UART) {
      flush_flash_to_host();
      xTaskNotify(
        control_task_handle, 
        NOTIFY_UPLOAD_DONE, 
        eSetBits
      ); 
    }

    if (notify_val & NOTIFY_UPLOAD_BLE) {
      // BLE transmission logic
    }
  }
}


/**
 * Sensor Capture Task and Timer ISR
 */ 
static void IRAM_ATTR sensor_timer_cb(void *arg) {
  BaseType_t hp_task_woken = pdFALSE; 
  xTaskNotifyFromISR(capture_task_handle, 0, eNoAction, &hp_task_woken); 

  if (hp_task_woken) {
    portYIELD_FROM_ISR(); 
  }
}
void capture_sensor_data_task(void *arg) {
  quat_sample_t quat_sample; 
  bool initialized = false; 
  std::chrono::system_clock::time_point before;
  std::chrono::system_clock::time_point now; 
  int64_t now_timestamp_us; 
  std::chrono::duration<float> dt; 

  for (;;) {
    // Wait for a notification from timer
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); 
    now = std::chrono::system_clock::now();
    if (!initialized) {
      before = now; 
      initialized = true; 
    } 

    dt = now - before;

    if (!imu.update(dt.count())) {
      ESP_LOGE("SENSOR_CAPTURE", "Failed to update an IMU."); 
      continue;
    }

    now_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()
    ).count();

    SENSORS::Imu::Quaternion sensor_quat = imu.get_orientation(); 

    quat_sample = {
      .quat = {.w = sensor_quat.w, .x = sensor_quat.x, .y = sensor_quat.y, .z = sensor_quat.z,}, 
      .timestamp_us = now_timestamp_us
    };

    if (xRingbufferSend(
      flash_writer_buf, 
      &quat_sample, 
      sizeof(quat_sample_t), 
      pdMS_TO_TICKS(2) // wait 2 ms for ring buffer to empty
    ) == pdFALSE) {
      ESP_LOGE("SENSOR_CAPTURE", "Failed sending sample to flash writer buffer."); 
    }

    before = now; 
  }
}

/**
 * Control Task Handle and Button ISR
 */
static Common::GPIO *boot_button = nullptr;
constexpr uint32_t BOOT_BUTTON_DEBOUNCE_MS = 200; 

static void IRAM_ATTR boot_button_isr(void *arg) {
  BaseType_t hp_task_woken = pdFALSE; 
  xTaskNotifyFromISR(
    control_task_handle, 
    NOTIFY_BUTTON_PRESS, 
    eSetBits, 
    &hp_task_woken
  ); 

  if (hp_task_woken) {
    portYIELD_FROM_ISR(); 
  }
}
void control_task(void *arg) {
  uint32_t notify_val; 
  uint32_t last_button_tick = xTaskGetTickCount();

  for (;;) {
    xTaskNotifyWait(
      0, 
      UINT32_MAX, 
      &notify_val, 
      portMAX_DELAY
    );

    // Button event
    if (notify_val & NOTIFY_BUTTON_PRESS) {
      uint32_t now = xTaskGetTickCount(); 
      uint32_t elapsed_ms = (now - last_button_tick) * portTICK_PERIOD_MS; 

      if (elapsed_ms < BOOT_BUTTON_DEBOUNCE_MS) {
        ESP_LOGD("CONTROL", "Button bounce ignored"); 
        continue; 
      }

      last_button_tick = now; 

      switch (system_state) {
      case system_state_idle: 
        ESP_LOGI("CONTROL", "IDLE -> RECORDING"); 
        system_state = system_state_recording; 
        esp_timer_start_periodic(sensor_timer, sensor_polling_period); 
        break; 
      case system_state_recording: 
        ESP_LOGI("CONTROL", "RECORDING -> COMPLETE"); 
        system_state = system_state_complete; 
        esp_timer_stop(sensor_timer);
        break;
      case system_state_complete: 
        ESP_LOGI("CONTROL", "COMPLETE -> FLUSHING"); 
        system_state = system_state_flushing; 
        break;
      default: 
        break;
      }
    }

    // Flash done event
    if (notify_val & NOTIFY_FLASH_DONE && system_state == system_state_flushing) {
      ESP_LOGI("CONTROL", "FLUSHING -> UPLOADING"); 
      system_state = system_state_uploading; 
      xTaskNotify(
        upload_task_handle, 
        NOTIFY_UPLOAD_UART, 
        eSetBits
      );
    }

    // Upload done event
    if (notify_val & NOTIFY_UPLOAD_DONE && system_state == system_state_uploading) {
      ESP_LOGI("CONTROL", "UPLOADING -> COMPLETE"); 
      system_state = system_state_complete;
    }
  }
}

/**
 * Initialization of the timers
 */
esp_err_t initTimers() {
  esp_timer_create_args_t sensor_timer_create_args = {
    .callback = sensor_timer_cb, 
    .arg = nullptr,

    .name = "sensor_timer", 
  }; 

  ESP_RETURN_ON_ERROR(
    esp_timer_create(&sensor_timer_create_args, &sensor_timer), 
    "TIMER_INIT", "Failed to create a sensor timer."
  ); 

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

  boot_button = 
    new Common::GPIO(GPIO_NUM_0, Common::GPIO::Direction::INPUT, Common::GPIO::Level::ON, Common::GPIO::PULLUP); 
  
  ESP_RETURN_ON_ERROR(
    boot_button->set_interrupt(Common::GPIO::INTERRUPT_FALLING_EDGE, boot_button_isr, nullptr), 
    "SYS_INIT", "Failed to set interrupt for boot button."
  );
  logger.info("Enabled Boot Button");

  // Initialize LED
  LED::led red_led = LED::led(LED::RED_LED);

  std::error_code ec;
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
    return ESP_ERR_INVALID_STATE; 
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  // init imu
  bool imu_initialized = imu.init();
  // ensure imu is configured correctly
  if (!imu_initialized) {
    logger.error("Failed to initialize imu"); 
    return ESP_ERR_INVALID_STATE;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // give imu time to startup before first i2c read

  uint8_t test = imu.get_whoami();
  if (test != 0xEA) {
    logger.error("Invalid imu device id {}", test);
  }

  logger.info("Initialized imu"); 

  red_led.turn_on();

  spi_bus_config_t spi2_bus_config {
    .mosi_io_num = spi2_mosi, 
    .miso_io_num = spi2_miso, 
    .sclk_io_num = spi2_sck,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1, 
  }; 

  ESP_RETURN_ON_ERROR(
    spi_bus_initialize(SPI2_HOST, &spi2_bus_config, SPI_DMA_CH_AUTO), 
    "SYS_INIT", "Failed to initialize SPI_2 bus."
  ); 

  ESP_RETURN_ON_ERROR(
    spi_flash.init(), 
    "SYS_INIT", "Failed to initialize SPI flash."
  ); 

  ESP_RETURN_ON_ERROR(
    flash_log.init(), 
    "SYS_INIT", "Failed to initialize flash log."
  );

  ESP_RETURN_ON_ERROR(
    initTimers(), 
    "SYS_INIT", "Failed to initialize timers."
  ); 

  flash_writer_buf = xRingbufferCreate(
    32 * sizeof(quat_sample_t), 
    RINGBUF_TYPE_NOSPLIT
  );

  if (flash_writer_buf == nullptr) {
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
  ble_config.on_connect = [](NimBLEConnInfo &info) {
    printf("BLE: Client connected!\n");
  };
  ble_config.on_disconnect = [](NimBLEConnInfo &info, espp::BleGattServer::DisconnectReason reason) {
    printf("BLE: Client disconnected\n");
  };
  ble_config.on_authenticated = [](const NimBLEConnInfo &info) {
    printf("BLE: Client authenticated successfully\n");
  };

  // Create and initialize BLE module
  ble_module_ptr = new BLE::BleModule(ble_config);
  ESP_RETURN_ON_ERROR(
    ble_module_ptr->init(), 
    "SYS_INIT", "Failed to initialize BLE module"
  );

  // Set MTU for larger packets
  ESP_RETURN_ON_FALSE(
    NimBLEDevice::setMTU(247), ESP_ERR_INVALID_STATE,
    "SYS_INIT", "Failed to set MTU on NimBLEDevice."
  );

  logger.info("BLE module initialized successfully");

  // Start advertising
  ESP_RETURN_ON_ERROR(
    ble_module_ptr->start_advertising(), 
    "SYS_INIT", "Failed to start advertising"
  );

  logger.info("BLE advertising started. Device name: {}", ble_config.device_name);
  logger.info("Connect with your phone's BLE scanner app");

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  Main Loop
// ---------------------------------------------------------------------

BLE::FrameSample samples_to_send[24] = {
  {1, 0.0, 0.0, 0.0, 0.0}, {2, 0.0, 0.0, 0.0, 0.0}, 
  {3, 0.0, 0.0, 0.0, 0.0}, {4, 0.0, 0.0, 0.0, 0.0}, 
  {5, 0.0, 0.0, 0.0, 0.0}, {6, 0.0, 0.0, 0.0, 0.0}, 
  {7, 0.0, 0.0, 0.0, 0.0}, {8, 0.0, 0.0, 0.0, 0.0}, 
  {9, 0.0, 0.0, 0.0, 0.0}, {10, 0.0, 0.0, 0.0, 0.0}, 
  {11, 0.0, 0.0, 0.0, 0.0}, {12, 0.0, 0.0, 0.0, 0.0}, 
  {13, 0.0, 0.0, 0.0, 0.0}, {14, 0.0, 0.0, 0.0, 0.0}, 
  {15, 0.0, 0.0, 0.0, 0.0}, {16, 0.0, 0.0, 0.0, 0.0}, 
  {17, 0.0, 0.0, 0.0, 0.0}, {18, 0.0, 0.0, 0.0, 0.0}, 
  {19, 0.0, 0.0, 0.0, 0.0}, {20, 0.0, 0.0, 0.0, 0.0}, 
  {21, 0.0, 0.0, 0.0, 0.0}, {22, 0.0, 0.0, 0.0, 0.0}, 
  {23, 0.0, 0.0, 0.0, 0.0}, {24, 0.0, 0.0, 0.0, 0.0}, 
}; 

esp_err_t mainLoop() {
  // Timing state
  static int64_t last_send_time_us = 0;
  static int64_t last_state_log_us = 0;
  static int64_t last_battery_update_us = 0;
  static int64_t last_waiting_log_us = 0;
  static uint16_t frame_seq = 0; 
  static size_t sample_idx_to_send = 0; 

  static bool was_connected = false; 
    
  const int64_t ACK_TIMEOUT_US = 30'000'000;  // 30 seconds

  if (!ble_module_ptr) {
    logger.error("ble_module_ptr is NULL!");
    return ESP_ERR_INVALID_STATE;
  }

  int64_t now_us = esp_timer_get_time();
  bool is_connected = ble_module_ptr->is_connected();
  if (!is_connected) {
    if (was_connected) {
      logger.info("Device disconnected – resetting state");
      was_connected = false;
      battery_level = 100;
    }
    
    if (now_us - last_waiting_log_us >= 10'000'000) {  // Every 10 seconds
      last_waiting_log_us = now_us;
      logger.info("Waiting for BLE connection…");
    }

    return ESP_OK; 
  }

  if (!was_connected) {
    logger.info("Device connected! Getting device info…");
    auto devs = ble_module_ptr->get_connected_device_infos();
    for (const auto &d : devs) {
      logger.info("Device: {}, RSSI: {} dBm", ble_module_ptr->get_device_name(d), ble_module_ptr->get_rssi(d));
    }
    was_connected = true;
    ble_module_ptr->reset_ack_on_connect();
    last_send_time_us = now_us;
  }

  // ACK/timeout handling
  bool can_send = ble_module_ptr->is_ack_received();
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
    uint16_t sample_count = 24 - sample_idx_to_send >= BLE::SAMPLES_PER_FRAME 
      ? BLE::SAMPLES_PER_FRAME 
      : 24 - sample_idx_to_send; 

    uint16_t payload_len = sample_count * ((uint16_t) sizeof(BLE::FrameSample)); 
    
    BLE::Frame frame {
      .header = {
        .frame_seq = frame_seq++,
        .sample_count = sample_count,
        .payload_len = payload_len,  
        .flags = 0, 
      }, 
      .payload = {}
    };

    memcpy(frame.payload, &samples_to_send[sample_idx_to_send], payload_len); 

    logger.info("Sending bulk frame seq {} (244 B)", (unsigned int) frame.header.frame_seq);
    ble_module_ptr->notify_quaternion(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    ble_module_ptr->reset_ack();
    last_send_time_us = now_us;

    sample_idx_to_send = sample_count + sample_idx_to_send >= 24 ? 0 : sample_count + sample_idx_to_send; 

    // Re-evaluate send condition
    can_send = ble_module_ptr->is_ack_received();
    timeout = false;
  }

  // ---------------------------------------------------------------------
  // Battery update (every 50 seconds)
  // ---------------------------------------------------------------------
  if (now_us - last_battery_update_us >= 50'000'000) {
    last_battery_update_us = now_us;
    ble_module_ptr->set_battery_level(battery_level);
    battery_level = (battery_level == 0) ? 100 : battery_level - 1;
    logger.info("Battery level: {}%", battery_level);
  }

  return ESP_OK;
}

// ---------------------------------------------------------------------
//  Application Entry Point
// ---------------------------------------------------------------------

extern "C" void app_main() {
  if (initSystem() != ESP_OK) {
    logger.error("Failed initializing a system.");
  } // called once

  // Create a task that will write samples to the flash
  xTaskCreate(
    flash_writer_task, 
    "flash_writer", 
    4096, 
    NULL, 
    3, 
    &flash_writer_task_handle
  );

  // Create a task that will capture sensor data
  xTaskCreate(
    capture_sensor_data_task, 
    "capture_sensor_data", 
    4096, 
    NULL, 
    8, 
    &capture_task_handle
  );

  // Create a task that will control program flow
  xTaskCreate(
    control_task, 
    "control", 
    4096, 
    NULL, 
    9, 
    &control_task_handle
  );

  // Create a task that will transmit data
  xTaskCreate(
    upload_task, 
    "upload", 
    4096, 
    NULL, 
    5, 
    &upload_task_handle
  ); 

  while (true) {
    mainLoop(); 
    std::this_thread::sleep_for(10ms); 
  }
}
