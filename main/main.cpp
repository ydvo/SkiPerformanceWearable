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
constexpr uint64_t sensor_polling_period{100000}; // 10 ms = 10_000 us

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
  system_state_flushing
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

static TaskHandle_t capture_task_handle = nullptr;

static Common::GPIO *boot_button = nullptr; 

static void IRAM_ATTR sensor_timer_cb(void *arg) {
  BaseType_t hp_task_woken = pdFALSE; 
  xTaskNotifyFromISR(capture_task_handle, 0, eNoAction, &hp_task_woken); 

  if (hp_task_woken) {
    portYIELD_FROM_ISR(); 
  }
}

/*
  capture_sensor_data()
    - updates an IMU
    - stores quaternion into a flash
*/
void capture_sensor_data_task(void *arg) {
  SENSORS::Imu::Quaternion quat; 
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

    if (imu.update(dt.count())) {
      quat = imu.get_orientation();
    } else {
      ESP_LOGE("SENSOR_CAPTURE", "Failed to update an IMU."); 
    }

    now_timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()
    ).count();

    if (flash_log.append(quat, now_timestamp_us) != ESP_OK) {
      ESP_LOGE("SENSOR_CAPTURE", "Failed appending to the flash log."); 
    }

    before = now; 
  }
}

void flush_flash_to_host() {
  STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame frames[5]; 
  size_t frames_read;

  for (;;) {
    if (flash_log.read(frames, 5, &frames_read) != ESP_OK) {
      printf("ERROR\n"); 
      break; 
    }

    if (frames_read == 0) {
      printf("END\n"); 
      break; 
    }

    for (size_t i = 0; i < frames_read; ++i) {
      auto &frame = frames[i];

      printf("%lld", frame.payload.start_t_us); 
      for (int j = 0; j < STORAGE::SAMPLES_PER_FRAME; ++j) {
        printf(",%.6f,%.6f,%.6f,%.6f",frame.payload.data->w, frame.payload.data->x, frame.payload.data->y, frame.payload.data->z);
      }
      printf(",%lld\n", frame.payload.end_t_us);
    }
  }
}

static TaskHandle_t control_task_handle = nullptr; 

static void IRAM_ATTR boot_button_isr(void *arg) {
  BaseType_t hp_task_woken = pdFALSE; 
  xTaskNotifyFromISR(control_task_handle, 0, eNoAction, &hp_task_woken); 

  if (hp_task_woken) {
    portYIELD_FROM_ISR(); 
  }
}

void control_task(void *arg) {
  for (;;) {
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    switch (system_state) {
    case system_state_idle: 
      system_state = system_state_recording;
      esp_timer_start_periodic(sensor_timer, sensor_polling_period);
      break; 
    case system_state_recording: 
      system_state = system_state_flushing;
      esp_timer_stop(sensor_timer); 
      flush_flash_to_host(); 
      system_state = system_state_idle; 
      break; 
    case system_state_flushing: 
    default: 
      break; 
    }
  }
}

/*
  initTimers()
    - initialization of sensor timer
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

  return ESP_OK; 
}

/* Application Entry Point */
extern "C" void app_main() {
  if (initSystem() != ESP_OK) {
    logger.error("Failed initializing a system.");
  } // called once

  // Create a task that will capture sensor data
  xTaskCreate(
    capture_sensor_data_task, 
    "capture_sensor_data", 
    4096, 
    NULL, 
    8, 
    &capture_task_handle
  );

  xTaskCreate(
    control_task, 
    "control", 
    4096, 
    NULL, 
    9, 
    &control_task_handle
  );

  while (true) {
    std::this_thread::sleep_for(1s); 
  }
}
