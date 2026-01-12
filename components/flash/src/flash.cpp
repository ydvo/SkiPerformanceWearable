#include "flash.hpp"

#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "driver/spi_common.h"
#include "esp_log.h"

namespace STORAGE {
  SpiFlash::SpiFlash(const SpiFlash::SpiConfig &config) {
    spi_bus_config_t bus_config {
      .mosi_io_num = config.mosi_port, 
      .miso_io_num = config.miso_port, 
      .sclk_io_num = config.sck_port,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1, 
    }; 

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO)); 

    esp_flash_t *flash_dev {nullptr}; 
    esp_flash_spi_device_config_t flash_dev_config {
      .host_id = SPI2_HOST, 
      .cs_io_num = config.spi_cs_port, 
      .io_mode = SPI_FLASH_SLOWRD,
      .input_delay_ns = 0,
      .freq_mhz = 20, 
    }; 
    ESP_ERROR_CHECK(spi_bus_add_flash_device(&flash_dev, &flash_dev_config)); 
    ESP_ERROR_CHECK(esp_flash_init(flash_dev));
  }
}