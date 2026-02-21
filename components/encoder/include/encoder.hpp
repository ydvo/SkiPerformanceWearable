/* encoder.hpp
 *  - Quadrature rotary encoder driver using lookup table decoding
 *  - Interrupt-driven on both channels (any-edge)
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "GPIO.hpp"
#include "esp_attr.h"
#include "esp_err.h"

namespace SENSORS {

class Encoder {
public:
  /* Constructor
   *  Takes:
   *    - pin_a:  GPIO for encoder channel A
   *    - pin_b:  GPIO for encoder channel B
   *    - cpr:    counts per revolution (after 4x decoding)
   */
  Encoder(gpio_num_t pin_a, gpio_num_t pin_b, uint32_t cpr);

  /* init
   *  Attaches any-edge interrupts on both channels
   *  Returns:
   *    - ESP_OK on success
   */
  esp_err_t init();

  /* get_count
   *  Returns the raw tick count (positive = CW, negative = CCW)
   */
  int32_t get_count() const;

  /* get_angle
   *  Returns the current angle in degrees [0, 360)
   */
  float get_angle() const;

  /* reset
   *  Zeros the tick count
   */
  void reset();

private:
  static IRAM_ATTR void encoder_isr(void *arg);

  Common::GPIO pin_a_;
  Common::GPIO pin_b_;
  uint32_t cpr_;

  volatile uint8_t enc_val_ = 0;
  std::atomic<int32_t> enc_count_{0};
};

} // namespace SENSORS
