#include "I2C.hpp"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstring>

namespace Common {

static const char *TAG = "I2C";

// Contstructor
I2C::I2C(i2c_port_t port, gpio_num_t sda, gpio_num_t scl) : port(port) {
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
  ESP_LOGI(TAG, "Deleting I2C driver on port %d", port);
  i2c_del_master_bus(this->bus_handle);
}

/* write
 *  - initiates an i2c write to device
 */
esp_err_t I2C::write(uint8_t device_addr, uint8_t reg, const uint8_t *data, size_t len, int freq) {
  // create device cfg
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = device_addr;
  dev_cfg.scl_speed_hz = freq;

  // get device handle and assign to master
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

  // prepare data
  i2c_master_transmit_multi_buffer_info_t tx_buffers[NUMBUFFERS] = {
      {.write_buffer = &reg, .buffer_size = BYTELENGTH},
      {.write_buffer = (uint8_t *)data, .buffer_size = len}};

  // transmit
  esp_err_t ret = i2c_master_multi_buffer_transmit(dev_handle, tx_buffers, 2, -1);

  // check for success
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "I2C write failed to 0x%02X: %s", device_addr, esp_err_to_name(ret));
  else
    ESP_LOGD(TAG, "I2C write to 0x%02X successful", device_addr);

  return ret;
}

/* read
 *  - initiates i2c read from device
 */
esp_err_t I2C::read(uint8_t device_addr, uint8_t reg, uint8_t *data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);

  if (len > 1)
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK)
    ESP_LOGE(TAG, "I2C read failed from 0x%02X: %s", device_addr, esp_err_to_name(ret));
  else
    ESP_LOGD(TAG, "I2C read from 0x%02X successful", device_addr);

  return ret;
}

} // namespace Common
