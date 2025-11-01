/* led.cpp
 *  implementation of simple led driver
 */
#include "led.hpp"
#include "GPIO.hpp"
#include "esp_err.h"
#include "esp_log.h"

using namespace LED;

static const char *TAG{"LED"};

led::led(gpio_num_t pin) : led_{pin, Common::GPIO::Direction::OUTPUT} {}

/* turn_on
 *  turns on led
 */
esp_err_t led::turn_on() {
  ESP_LOGI(TAG, "LED ON");
  return led_.setLevel(LED_ON);
}

/* turn_off
 *  turns off led
 */
esp_err_t led::turn_off() {
  ESP_LOGI(TAG, "LED OFF");
  return led_.setLevel(LED_OFF);
}
