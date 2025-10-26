/* I2C.h
 *  - wrappers for esp-idf i2c functions
 */

#pragma once
#include "driver/i2c_master.h"
#include <cstdint>

// Part of common wrapper functions
namespace Common {
// constants
constexpr int ICM20948_I2C_HZ{400000};
constexpr uint8_t NUMBUFFERS{2};
constexpr uint8_t BYTELENGTH{1};

class I2C {
public:
  I2C(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
  ~I2C();

  /* write
   *  - initiates an i2c write to device
   *  Takes:
   *    - device address, register to write to, data pointer, size of data
   *  Returns:
   *    - Status
   */
  esp_err_t write(uint8_t device_addr, uint8_t reg, const uint8_t *data, size_t len,
                  int freq = ICM20948_I2C_HZ);

  /* read
   *  - initiates an i2c read to device
   *  Takes:
   *    - device address, register to read, data pointer, size of data
   *  Returns:
   *    - Status
   */
  esp_err_t read(uint8_t device_addr, uint8_t reg, uint8_t *data, size_t len);

private:
  i2c_port_t port;                    // i2c port
  i2c_master_bus_handle_t bus_handle; // master handle for i2c
};

} // namespace Common
