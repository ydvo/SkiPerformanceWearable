/* I2C.h
 *  - wrappers for esp-idf i2c functions
 */

#pragma once
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include <cstdint>
#include <vector>

namespace Common {
// constants
constexpr int ICM20948_I2C_HZ{400000};
constexpr uint8_t NUMBUFFERS{2};
constexpr uint8_t BYTELENGTH{1};
constexpr uint32_t I2CTIMEOUT_MS{1000};

struct I2CDevice {
  uint8_t address;
  uint32_t freq_hz;
  i2c_master_dev_handle_t handle;
};

class I2C {
public:
  I2C(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
  ~I2C();

  /* init_device
   *  - registers i2c device with master handler
   *  Takes:
   *    - device address
   *    - device i2c frequency
   *  Returns:
   *    - Status
   */
  esp_err_t init_device(uint8_t device_addr, uint32_t freq);

  /* reg_write
   *  - performs an i2c write to device register
   *  Takes:
   *    - device address, register to write to, data pointer, size of data
   *  Returns:
   *    - Status
   *  Notes:
   *    - Ensure device is initiliazed before
   */
  esp_err_t reg_write(uint8_t device_addr, uint8_t reg, const uint8_t *data, size_t len);

  /* reg_read
   *  - performs an i2c read from device register
   *  Takes:
   *    - device address, register to read, data pointer, size of data
   *  Returns:
   *    - Status
   *  Notes:
   *    - Ensure device is initiliazed before
   */
  esp_err_t reg_read(uint8_t device_addr, uint8_t reg, uint8_t *buffer, size_t len);

private:
  i2c_master_bus_handle_t bus_handle_; // master handle for i2c
  std::vector<I2CDevice> devices_;

  /* get_device_handle
   *  - retrieves device handle from stored devices
   */
  i2c_master_dev_handle_t get_device_handle(uint8_t device_addr);
};

} // namespace Common
