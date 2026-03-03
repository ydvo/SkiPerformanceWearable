/**
 * @file led.hpp
 * @ingroup feedback
 * @brief Simple LED driver.
 *
 * Provides a lightweight wrapper around a GPIO pin for turning an LED on or
 * off. Uses the `Common::GPIO` class for low‑level pin control.
 */
#pragma once
#include "GPIO.hpp"
#include "esp_err.h"

namespace LED {

/* Constants */
constexpr gpio_num_t GREEN_LED{GPIO_NUM_9};
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

  /** toggle
   * Returns: 
   *      -Status
   */
  esp_err_t toggle(); 

private:
  Common::GPIO led_;
};
} // namespace LED
