#include "I2C.hpp"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstddef>
#include <cstring>

/* Implementation */
namespace Common {

static const char *TAG = "I2C";

// Contstructor
I2C::I2C(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) {
  ESP_LOGI(TAG, "Initializing I2C on port %d (SDA=%d, SCL=%d)", port, sda, scl);

  // populate master cfg struct with mandatory parameters
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.i2c_port = port;
  i2c_mst_config.scl_io_num = scl;
  i2c_mst_config.sda_io_num = sda;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.flags.enable_internal_pullup = false;

  // get master handle and store
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
}

// Destructor
I2C::~I2C() {
  ESP_LOGI(TAG, "Deleting I2C driver");

  // remove devices
  for (I2CDevice &dev : devices) {
    i2c_master_bus_rm_device(dev.handle);
  }

  // remove master handle
  i2c_del_master_bus(this->bus_handle);
}

/* init_device
 *  - registers device with master handler and adds to cache
 */
esp_err_t I2C::init_device(uint8_t device_addr, uint32_t freq) {
  // create device cfg
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = device_addr;
  dev_cfg.scl_speed_hz = freq;

  // get device handle and assign to master
  i2c_master_dev_handle_t dev_handle;
  esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);

  if (ret == ESP_OK) {
    // add device to cache
    devices.push_back({device_addr, freq, dev_handle});
  }

  return ret;
}

/* get_device_handle
 *  - retrieves device handle from cache
 */
i2c_master_dev_handle_t I2C::get_device_handle(uint8_t device_addr) {
  for (I2CDevice &dev : devices) {
    if (dev.address == device_addr)
      return dev.handle;
  }

  // if no device found return null
  return nullptr;
}

/* reg_write
 *  - initiates an i2c write to device register
 *  - Sends Address + Write, then Reg Address, then Data
 */
esp_err_t I2C::reg_write(uint8_t device_addr, uint8_t reg, const uint8_t *data, size_t len) {

  // get device handle
  i2c_master_dev_handle_t dev_handle = get_device_handle(device_addr);

  // prepare data
  i2c_master_transmit_multi_buffer_info_t tx_buffers[NUMBUFFERS] = {
      {.write_buffer = &reg, .buffer_size = sizeof(reg)},
      {.write_buffer = (uint8_t *)data, .buffer_size = len}};

  // transmit
  esp_err_t ret =
      i2c_master_multi_buffer_transmit(dev_handle, tx_buffers, NUMBUFFERS, I2CTIMEOUT_MS);

  // check for success
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "I2C write failed to 0x%02X: %s", device_addr, esp_err_to_name(ret));
  else
    ESP_LOGD(TAG, "I2C write to 0x%02X successful", device_addr);

  return ret;
}

/* reg_read
 *  - initiates i2c read from device
 *  - Sends Address + Write, then Register Address, then Reads to buffer
 */
esp_err_t I2C::reg_read(uint8_t device_addr, uint8_t reg, uint8_t *buffer, size_t len) {

  // get device handle
  i2c_master_dev_handle_t dev_handle = get_device_handle(device_addr);

  // check that device handle exists
  if (!dev_handle) {
    ESP_LOGE(TAG, "Device 0x%02X not initialized", device_addr);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret =
      i2c_master_transmit_receive(dev_handle, &reg, sizeof(reg), buffer, len, I2CTIMEOUT_MS);

  if (ret != ESP_OK)
    ESP_LOGE(TAG, "I2C read failed from 0x%02X: %s", device_addr, esp_err_to_name(ret));
  else
    ESP_LOGD(TAG, "I2C read from 0x%02X successful", device_addr);

  return ret;
}

} // namespace Common
