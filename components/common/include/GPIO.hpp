/* GPIO.hpp
 *  - wrappers for esp-idf i2c functions
 */

#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include "soc/gpio_num.h"

namespace Common {

class GPIO {
public:
  enum Direction {
    INPUT = GPIO_MODE_INPUT,
    OUTPUT = GPIO_MODE_OUTPUT,
    INPUT_OUTPUT = GPIO_MODE_INPUT_OUTPUT
  };

  enum Pull {
    NONE = 0,
    PULLUP = GPIO_PULLUP_ONLY,
    PULLDOWN = GPIO_PULLDOWN_ONLY,
  };
  enum InterruptType {
    INTERRUPT_NONE = GPIO_INTR_DISABLE,
    INTERRUPT_RISING_EDGE = GPIO_INTR_POSEDGE,
    INTERRUPT_FALLING_EDGE = GPIO_INTR_NEGEDGE,
    INTERRUPT_ANY_EDGE = GPIO_INTR_ANYEDGE
  };

  explicit GPIO(gpio_num_t pin, Direction dir = INPUT, Pull pull = NONE,
                bool initial_level = false);
  ~GPIO();

  /* setLevel
   *  set GPIO output level
   *  Takes:
   *    - output level ( 0 or 1 )
   *  Returns:
   *    - Status
   */
  esp_err_t setLevel(bool level);

  /* getLevel
   *  gets GPIO output level
   *  Returns:
   *    - output level ( 0 or 1 )
   */
  bool getLevel() const;

  /* getPin
   *  gets Pin associated with GPIO instance
   *  Returns:
   *    - pin
   */
  gpio_num_t getPin() const;

  /* set_interrupt
   *  enable interrup, initiliaze isr and attach handler for GPIO pin
   *  Takes:
   *    - Interrupt Type
   *    - Handler function
   *    - void*
   *  Returns:
   *    - Status
   */
  esp_err_t set_interrupt(InterruptType type, gpio_isr_t handler, void *arg = nullptr);

  /* disable_interrupt
   *  frees handler from isr and disables interrupt on pin
   */
  esp_err_t disable_interrupt();

private:
  gpio_num_t pin_;
  bool interrupt_enabled_ = false;
};

} // namespace Common
