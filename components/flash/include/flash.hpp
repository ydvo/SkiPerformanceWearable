#pragma once
#include "soc/gpio_num.h"

namespace STORAGE {
class SpiFlash {
public: 
  struct SpiConfig {
    gpio_num_t sck_port;
    gpio_num_t mosi_port; 
    gpio_num_t miso_port;
    gpio_num_t spi_cs_port;
  };

  explicit SpiFlash(const SpiConfig& config); 
}; 
}