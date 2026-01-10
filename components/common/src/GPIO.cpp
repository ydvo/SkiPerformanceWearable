/* GPIO.cpp
 *  - wrappers for esp-idf i2c functions
 */

#include "GPIO.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/gpio_types.h"

namespace Common {

static const char *TAG = "GPIO";

/* Constructer
 *  Takes:
 *    - Pin number
 *    - GPIO mode ( Input or Output )
 */
GPIO::GPIO(gpio_num_t pin, Direction dir, bool initial_level, Pull pull) : pin_(pin) {
  ESP_LOGI(TAG, "Configuring GPIO %d", pin_);

  // configure pin with default no interupts or pull up/down
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = 1ULL << pin;
  io_conf.mode = static_cast<gpio_mode_t>(dir);
  io_conf.pull_up_en = (pull == PULLUP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = (pull == PULLUP) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;

  // Log error if issues initiliazing
  esp_err_t ret = gpio_config(&io_conf);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "GPIO %d config failed: %s", pin_, esp_err_to_name(ret));

  // set initial pin level
  if (dir == OUTPUT || dir == INPUT_OUTPUT) {
    gpio_set_level(pin_, initial_level);
  }
}

// Destructor
GPIO::~GPIO() {
  // if interrupt registered, remove it
  if (interrupt_enabled_) {
    gpio_isr_handler_remove(pin_);
  }
}

/* setLevel
 *  set GPIO output level
 *  Takes:
 *    - output level ( 0 or 1 )
 */
esp_err_t GPIO::setLevel(bool level) {
  esp_err_t ret = gpio_set_level(pin_, level);
  ESP_LOGD(TAG, "GPIO %d set to %d", pin_, level);
  return ret;
}

/* getLevel
 *  gets GPIO output level
 *  Returns:
 *    - output level ( 0 or 1 )
 */
bool GPIO::getLevel() const {
  bool val = gpio_get_level(pin_);
  ESP_LOGD(TAG, "GPIO %d read as %d", pin_, val);
  return val;
}

/* getPin
 *  Returns:
 *    - pin
 */
gpio_num_t GPIO::getPin() const {
  return this->pin_;
}

/* set_interrupt
 *  enable interrup, initiliaze isr and attach handler for GPIO pin
 *  Takes:
 *    - Interrupt Type
 *    - Handler function
 *    - void*
 *  Returns:
 *    - Status
 */
esp_err_t GPIO::set_interrupt(GPIO::InterruptType type, gpio_isr_t handler, void *arg) {
  esp_err_t ret = gpio_set_intr_type(pin_, static_cast<gpio_int_type_t>(type));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error setting interrupt type for GPIO %d", pin_);
    return ret;
  }

  // Enable isr globally if first interrupt pin
  static bool isr_service_installed = false;
  if (!isr_service_installed) {
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Error initiliazing ISR");
      return ret;
    }
    isr_service_installed = true;
  }

  // attach handler
  ret = gpio_isr_handler_add(pin_, handler, arg ? arg : (void *)this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error attaching interrupt for GPIO %d", pin_);
    return ret;
  }
  // indicate that pin has interrupt
  interrupt_enabled_ = true;
  return ESP_OK;
}

/* disable_interrupt
 *  frees handler from isr and disables interrupt on pin
 *  Returns:
 *    - Status
 */
esp_err_t GPIO::disable_interrupt() {
  // free handler from isr
  esp_err_t ret = gpio_isr_handler_remove(pin_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error removing interrupt handler on GPIO %d", pin_);
    return ret;
  }
  // disable interrupt on pin
  ret = gpio_set_intr_type(pin_, GPIO_INTR_DISABLE);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Error disabling interrupt on GPIO %d", pin_);
    return ret;
  }
  interrupt_enabled_ = false;

  return ESP_OK;
}

} // namespace Common
