#include "flash.hpp"

#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_check.h"

namespace STORAGE {

  static const char *TAG = "SPI_FLASH_DEVICE";

  SpiFlashDevice::SpiFlashDevice(const SpiFlashDevice::SpiConfig &config) noexcept: spi_config_ {config}, flash_dev_ {nullptr}, initialized_ {false} {}; 

  esp_err_t SpiFlashDevice::init() {
    if (initialized_) return ESP_OK; 

    spi_bus_config_t bus_config {
      .mosi_io_num = spi_config_.mosi_port, 
      .miso_io_num = spi_config_.miso_port, 
      .sclk_io_num = spi_config_.sck_port,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1, 
    }; 

    ESP_RETURN_ON_ERROR(
      spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO), 
      TAG, "spi bus initialization failed"
    ); 

    esp_flash_spi_device_config_t flash_dev_config {
      .host_id = SPI2_HOST, 
      .cs_io_num = spi_config_.spi_cs_port, 
      .io_mode = SPI_FLASH_SLOWRD,
      .input_delay_ns = 0,
      .freq_mhz = 20, 
    }; 

    ESP_RETURN_ON_ERROR(
      spi_bus_add_flash_device(&flash_dev_, &flash_dev_config), 
      TAG, "add flash device failed"
    ); 
    ESP_RETURN_ON_ERROR(
      esp_flash_init(flash_dev_), 
      TAG, "flash initialization failed"
    );
    ESP_LOGI("SPI_FLASH_DEVICE", "Initialized SPI Flash Device (id: 0x%x, sz: %d)", flash_dev_->chip_id, flash_dev_->size); 

    initialized_ = true; 

    return ESP_OK; 
  }
}