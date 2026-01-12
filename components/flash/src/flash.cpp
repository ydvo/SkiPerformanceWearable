#include "flash.hpp"

#include "esp_flash.h"
#include "spi_flash_chip_driver.h"
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

  esp_err_t SpiFlashDevice::check_bounds(const uint32_t addr, size_t len) const noexcept {
    if (!initialized_ || flash_dev_ != nullptr || addr + len > flash_dev_->size) {
      return ESP_ERR_INVALID_STATE; 
    }

    return ESP_OK; 
  }

  esp_err_t SpiFlashDevice::read(const uint32_t addr, void *dst, size_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE; 

    ESP_RETURN_ON_ERROR(
      check_bounds(addr, len), 
      TAG, "read outside of bounds"
    ); 
    
    return esp_flash_read(flash_dev_, dst, addr, len); 
  }

  esp_err_t SpiFlashDevice::write(const uint32_t addr, void *src, size_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE; 

    ESP_RETURN_ON_ERROR(
      check_bounds(addr, len), 
      TAG, "write outside of bounds"
    ); 

    return esp_flash_write(flash_dev_, src, addr, len); 
  }

  esp_err_t SpiFlashDevice::erase(){
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    return esp_flash_erase_chip(flash_dev_); 
  }

  esp_err_t SpiFlashDevice::erase_region(const uint32_t addr, size_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE; 
    
    if (addr % flash_dev_->chip_drv->sector_size != 0 || len % flash_dev_->chip_drv->sector_size != 0) {
      return ESP_ERR_INVALID_ARG; 
    }

    ESP_RETURN_ON_ERROR(
      check_bounds(addr, len), 
      TAG, "erase outside of bounds"
    );

    return esp_flash_erase_region(flash_dev_, addr, len); 
  }
}