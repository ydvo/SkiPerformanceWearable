/* led.hpp
 *  simple driver for leds
 */
#pragma once
#include "GPIO.hpp"
#include "esp_err.h"

namespace LED {

/* Constants */
constexpr gpio_num_t RED_LED{GPIO_NUM_13};
constexpr bool LED_OFF{0};
constexpr bool LED_ON{1};

class led {

public:
  led(gpio_num_t pin);

  /* turn_on
   *  Returns:
   *      -Status
   */
  esp_err_t turn_on();

  /* turn_off
   * Returns:
   *      -Status
   */
  esp_err_t turn_off();

private:
  Common::GPIO led_;
};
} // namespace LED
