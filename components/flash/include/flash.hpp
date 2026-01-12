#pragma once
#include "soc/gpio_num.h"
#include "logger.hpp"
#include "esp_flash.h"

namespace STORAGE {
class SpiFlashDevice {
public:
  struct SpiConfig {
    gpio_num_t sck_port;
    gpio_num_t mosi_port; 
    gpio_num_t miso_port;
    gpio_num_t spi_cs_port;
  };

  explicit SpiFlashDevice(const SpiConfig& config) noexcept; 
  esp_err_t init(); 
  esp_err_t check_bounds(const uint32_t addr, size_t len) const noexcept; 
  esp_err_t read(const uint32_t addr, void *dst, size_t len); 
  esp_err_t write(const uint32_t addr, void *src, size_t len); 
  esp_err_t erase(); 
  esp_err_t erase_region(const uint32_t addr, size_t len); 

private: 
  SpiConfig spi_config_;
  esp_flash_t *flash_dev_;
  bool initialized_; 
}; 
}