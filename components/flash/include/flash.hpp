/**
 * @file flash.hpp
 * @ingroup storage
 * @brief SPI flash device wrapper.
 *
 * Provides a thin C++ wrapper around the ESP‑IDF SPI flash APIs. It manages
 * initialization, bounds‑checked reads/writes, and sector/ chip erase operations.
 */
#pragma once
#include "soc/gpio_num.h"
#include "logger.hpp"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"

namespace STORAGE {
/**
 * @brief SPI flash device wrapper.
 *
 * Encapsulates an ESP‑IDF `esp_flash_t` instance. Handles one‑time
 * initialization of the SPI flash peripheral, provides bounds‑checked reads
 * and writes, and supports chip‑wide or region‑based erase operations.
 *
 * Thread‑safety: not safe for concurrent access without external
 * synchronization; typical use is a single owner (e.g., the flash log).
 */
class SpiFlashDevice {
public:
  /**
   * @brief Configuration parameters for the SPI flash device.
+   *
+   * `host` selects the SPI host (e.g., SPI2_HOST) and `cs` specifies the GPIO
+   * used for Chip‑Select.
+   */
  struct Config {
    spi_host_device_t host; 
    gpio_num_t cs;
  };

  /**
   * @brief Construct a SpiFlashDevice with the given configuration.
+   *
+   * Does not perform hardware initialization; call `init()` before any I/O.
+   *
+   * @param config Configuration struct containing host and CS pin.
+   */
  explicit SpiFlashDevice(const Config &config) noexcept;
  /**
   * @brief Initialize the SPI flash peripheral.
+   *
+   * Performs one‑time setup of the SPI bus and attaches the flash device.
+   * Subsequent calls are no‑ops and return ESP_OK.
+   *
+   * @return ESP_OK on success, otherwise an ESP‑IDF error code.
+   */
  esp_err_t init();
  /**
   * @brief Read bytes from flash.
+   *
+   * @param addr  Flash address (byte offset) to start reading from.
+   * @param dst   Destination buffer (must be at least `len` bytes).
+   * @param len   Number of bytes to read.
+   * @return ESP_OK on success, or an error if the device is not initialized
+   *         or the range exceeds flash bounds.
+   */
  esp_err_t read(const uint32_t addr, void *dst, size_t len);
  /**
   * @brief Write bytes to flash.
+   *
+   * @param addr  Flash address to write to.
+   * @param src   Source buffer containing data to write.
+   * @param len   Number of bytes to write.
+   * @return ESP_OK on success, otherwise an error (e.g., out of bounds or device
+   *         not initialized).
+   */
  esp_err_t write(const uint32_t addr, const void *src, size_t len);
  /**
   * @brief Erase the entire flash chip.
+   *
+   * This operation destroys all data on the device.
+   * @return ESP_OK on success, otherwise an error code.
+   */
  esp_err_t erase_chip();
  /**
   * @brief Erase a region of flash.
+   *
+   * @param addr  Starting address; must be sector‑aligned.
+   * @param len   Length in bytes; must be a multiple of sector size.
+   * @return ESP_OK on success, otherwise ESP_ERR_INVALID_ARG if alignment is
+   *         incorrect or ESP_ERR_INVALID_STATE if device not initialized.
+   */
  esp_err_t erase_region(const uint32_t addr, size_t len);

  /**
   * @brief Get total flash size in bytes.
+   * @return Flash capacity.
+   */
  size_t size_bytes() const;
  /**
   * @brief Get flash sector size in bytes.
+   * @return Size of a flash erase sector.
+   */
  size_t sector_size() const;
  /**
   * @brief Get flash page size in bytes.
+   * @return Size of a programmable page.
+   */
  size_t page_size() const;

private: 
  Config config_;
  esp_flash_t flash_{};
  const spi_flash_chip_t *chip_{}; 
  bool initialized_ {false}; 

  esp_err_t check_bounds(const uint32_t addr, size_t len) const noexcept;
}; 
}