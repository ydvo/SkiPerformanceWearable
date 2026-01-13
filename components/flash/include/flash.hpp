#pragma once
#include "soc/gpio_num.h"
#include "logger.hpp"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"

namespace STORAGE {
class SpiFlashDevice {
public:
  struct Config {
    spi_host_device_t host; 
    gpio_num_t cs;
  };

  explicit SpiFlashDevice(const Config &config) noexcept; 
  esp_err_t init();  
  esp_err_t read(const uint32_t addr, void *dst, size_t len); 
  esp_err_t write(const uint32_t addr, const void *src, size_t len); 
  esp_err_t erase_chip(); 
  esp_err_t erase_region(const uint32_t addr, size_t len); 

  size_t size_bytes() const; 
  size_t sector_size() const; 
  size_t page_size() const; 

private: 
  Config config_;
  esp_flash_t flash_{};
  const spi_flash_chip_t *chip_{}; 
  bool initialized_ {false}; 

  esp_err_t check_bounds(const uint32_t addr, size_t len) const noexcept;
}; 
}