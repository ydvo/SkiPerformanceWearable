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


#include <cstdio>
#include <stdint.h>
#include <array>

using namespace std::chrono_literals;

static esp_timer_handle_t sensor_timer; 
constexpr uint64_t sensor_polling_period{10000}; // 10 ms = 10_000 us

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// spi 2 pins
static constexpr auto spi2_sck {GPIO_NUM_12}; 
static constexpr auto spi2_mosi {GPIO_NUM_11}; 
static constexpr auto spi2_miso {GPIO_NUM_13}; 

static constexpr auto flash_spi_cs {GPIO_NUM_10}; 

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
STORAGE::SpiFlashDevice spi_flash ({
  .host = SPI2_HOST,
  .cs = flash_spi_cs
});

// Flash Log
STORAGE::FlashLog<SENSORS::Imu::Quaternion> flash_log(
  spi_flash
);
// IMU
SENSORS::Imu imu(i2c);

/**
 * Flushes the flash contents over UART
 */
void flush_flash_to_host() {
  STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame frames[5]; 
  size_t frames_read;
  uint32_t read_addr = flash_log.read_addr(); 
  uint32_t start_addr = read_addr; 

  printf("START\n"); 

  for (;;) {
    if (flash_log.read_immut(frames, 5, &frames_read, read_addr, &read_addr) != ESP_OK) {
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
  SENSORS::Imu::Quaternion quat; 
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

    quat_sample = {
      .quat = imu.get_orientation(), 
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

/*
 * initSystem()
 *  - initialization function, runs once before main loop
 */
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

  // led
  LED::led red_led = LED::led(LED::RED_LED);

  // create i2c instance
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);

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

  return ESP_OK; 
}

/* Application Entry Point */
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
    std::this_thread::sleep_for(1s); 
  }
}
