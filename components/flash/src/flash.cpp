/**
 * @file flash.cpp - SPI flash device implementation.
 *
 * Implements the `SpiFlashDevice` class methods for initializing the device,
 * performing bounds‑checked reads/writes, and erasing sectors or the entire
 * chip.
 */
#include "flash.hpp"
#include "flash_log.hpp"
#include "imu.hpp"

#include "esp_flash.h"
#include "spi_flash_chip_driver.h"
#include "esp_flash_spi_init.h"
#include "esp_log.h"
#include "esp_check.h"

namespace STORAGE {

  static const char *TAG = "SPI_FLASH_DEVICE";

  SpiFlashDevice::SpiFlashDevice(const SpiFlashDevice::Config &config) noexcept: config_{config} {}; 

  esp_err_t SpiFlashDevice::init() {
    if (initialized_) return ESP_OK;

    esp_flash_spi_device_config_t flash_dev_config {
      .host_id = config_.host, 
      .cs_io_num = config_.cs, 
      .io_mode = SPI_FLASH_SLOWRD,
      .input_delay_ns = 0,
      .freq_mhz = 20, 
    }; 

    esp_flash_t *flash_ptr = nullptr; 

    ESP_RETURN_ON_ERROR(
      spi_bus_add_flash_device(&flash_ptr, &flash_dev_config), 
      TAG, "add flash device failed"
    );

    ESP_RETURN_ON_ERROR(
      esp_flash_init(flash_ptr), 
      TAG, "flash initialization failed"
    );

    flash_ = *flash_ptr; 
    chip_ = flash_.chip_drv; 
    initialized_ = true; 

    ESP_LOGI(
      TAG, "Initialized SPI Flash Device (id: 0x%x, sz: %d, sector_sz: %d, page_size: %d)", 
      flash_.chip_id, flash_.size, chip_->sector_size, chip_->page_size
    ); 

    return ESP_OK; 
  }

  esp_err_t SpiFlashDevice::check_bounds(const uint32_t addr, size_t len) const noexcept {
    if (!initialized_ || addr + len > flash_.size) {
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
    
    return esp_flash_read(&flash_, dst, addr, len); 
  }

  esp_err_t SpiFlashDevice::write(const uint32_t addr, const void *src, size_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE; 

    ESP_RETURN_ON_ERROR(
      check_bounds(addr, len), 
      TAG, "write outside of bounds"
    ); 

    return esp_flash_write(&flash_, src, addr, len); 
  }

  esp_err_t SpiFlashDevice::erase_chip(){
    if (!initialized_) return ESP_ERR_INVALID_STATE;

    return esp_flash_erase_chip(&flash_); 
  }

  esp_err_t SpiFlashDevice::erase_region(const uint32_t addr, size_t len) {
    if (!initialized_) return ESP_ERR_INVALID_STATE; 

    if (addr % flash_.chip_drv->sector_size != 0 || len % flash_.chip_drv->sector_size != 0) {
      return ESP_ERR_INVALID_ARG; 
    }

    ESP_RETURN_ON_ERROR(
      check_bounds(addr, len), 
      TAG, "erase outside of bounds"
    );

    return esp_flash_erase_region(&flash_, addr, len); 
  }

  size_t SpiFlashDevice::size_bytes() const {
    return flash_.size; 
  }

  size_t SpiFlashDevice::sector_size() const {
    return chip_->sector_size; 
  }

  size_t SpiFlashDevice::page_size() const {
    return chip_->page_size; 
  }

  // Create the SPI flash device instance
  SpiFlashDevice flash_device({
      .host = SPI2_HOST,
      .cs = GPIO_NUM_10
  });

} // namespace STORAGE


extern STORAGE::FlashLog<SENSORS::Imu::Quaternion> flash_log;