/* GPIO.hpp
 *  - wrappers for esp-idf i2c functions
 */

#pragma once
#include "driver/gpio.h"
#include "soc/gpio_num.h"

namespace Common {

class GPIO {
public:
  GPIO(gpio_num_t pin, gpio_mode_t mode);
  ~GPIO();

  /* setLevel
   *  set GPIO output level
   *  Takes:
   *    - output level ( 0 or 1 )
   */
  void setLevel(uint8_t level);

  /* getLevel
   *  gets GPIO output level
   *  Returns:
   *    - output level ( 0 or 1 )
   */
  uint8_t getLevel() const;

  /* getPin
   *  gets Pin associated with GPIO instance
   *  Returns:
   *    - pin
   */
  gpio_num_t getPin() const;

private:
  gpio_num_t pin_;
};

} // namespace Common
