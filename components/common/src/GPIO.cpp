/* GPIO.cpp
 *  - wrappers for esp-idf i2c functions
 */

#include "GPIO.hpp"
#include "esp_log.h"
#include "soc/gpio_num.h"

namespace Common {

static const char *TAG = "GPIO";

/* Constructer
 *  Takes:
 *    - Pin number
 *    - GPIO mode ( Input or Output )
 */
GPIO::GPIO(gpio_num_t pin, gpio_mode_t mode) : pin_(pin) {
  ESP_LOGI(TAG, "Configuring GPIO %d", pin_);

  // configure pin with default no interupts or pull up/down
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = 1ULL << pin;
  io_conf.mode = mode;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;

  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "GPIO %d config failed: %s", pin_, esp_err_to_name(ret));
}

// Destructor
GPIO::~GPIO() {
  // nothing for now
}

/* setLevel
 *  set GPIO output level
 *  Takes:
 *    - output level ( 0 or 1 )
 */
void GPIO::setLevel(uint8_t level) {
  gpio_set_level(pin_, level);
  ESP_LOGD(TAG, "GPIO %d set to %d", pin_, level);
}

/* getLevel
 *  gets GPIO output level
 *  Returns:
 *    - output level ( 0 or 1 )
 */
uint8_t GPIO::getLevel() const {
  uint8_t val = gpio_get_level(pin_);
  ESP_LOGD(TAG, "GPIO %d read as %d", pin_, val);
  return val;
}

gpio_num_t GPIO::getPin() const {
  return this->pin_;
}

} // namespace Common
