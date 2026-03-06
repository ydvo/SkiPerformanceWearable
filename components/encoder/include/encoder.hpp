/**
 * @file encoder.hpp
 * @ingroup sensors
 * @brief Quadrature rotary encoder driver.
 *
 * Implements a high‑resolution rotary encoder using a lookup‑table decoder.
 * Both A and B channels are configured for any‑edge interrupts; the ISR
 * updates a thread‑safe tick count using `std::atomic`. The driver provides
 * raw tick count, angle in degrees, and reset functionality.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "GPIO.hpp"
#include "esp_attr.h"
#include "esp_err.h"

namespace SENSORS {

/**
 * @brief Quadrature encoder driver.
 *
 * The class configures two GPIO pins for a rotary encoder's A and B channels
 * and uses an ISR‑based lookup‑table algorithm to decode transitions. Tick count
 * is stored in a `std::atomic<int32_t>` to allow safe concurrent reads from
 * FreeRTOS tasks. Angle is computed on‑the‑fly from the count and the configured
 * counts‑per‑revolution (CPR).
 */
class Encoder {
public:
  /**
   * @brief Construct an Encoder instance.
+   *
+   * Configures two GPIO pins as inputs with internal pull‑up resistors and
+   * stores the counts‑per‑revolution value. No hardware is accessed until
+   * `init()` is called.
+   *
+   * @param pin_a GPIO number for channel A.
+   * @param pin_b GPIO number for channel B.
+   * @param cpr   Encoder counts per full mechanical revolution after 4×
+   *              quadrature decoding.
+   */
  Encoder(gpio_num_t pin_a, gpio_num_t pin_b, uint32_t cpr);

  /**
   * @brief Initialize the encoder hardware.
+   *
+   * Configures both channel GPIOs for any‑edge interrupts and registers the ISR.
+   * The initial pin states are sampled to seed the quadrature decoder.
+   *
+   * @return ESP_OK on success, otherwise an ESP‑IDF error code.
+   */
  esp_err_t init();

  /**
   * @brief Retrieve the raw tick count.
+   *
+   * Positive values indicate clockwise rotation, negative values indicate
+   * counter‑clockwise rotation.
+   *
+   * @return Current tick count (atomic read, relaxed ordering).
+   */
  int32_t get_count() const;

  /**
   * @brief Compute the current mechanical angle.
+   *
+   * Angle is calculated as `count * 360 / cpr` and wrapped to the range
+   * `[0, 360)`. The computation uses `fmodf` to handle rollover and corrects
+   * negative results by adding 360 degrees.
+   *
+   * @return Angle in degrees, wrapped to `[0, 360)`.
+   */
  float get_angle() const;

  /**
   * @brief Reset the encoder count to zero.
+   *
+   * The atomic counter is stored as zero; no hardware state is modified.
+   */
  void reset();

private:
  /**
   * @brief ISR for quadrature decoder.
+   *
+   * Invoked on any edge of either channel. It reads the current pin levels,
+   * updates the 4‑bit state history (`enc_val_`), looks up the transition delta
+   * from the static `lookup_table`, and atomically adds the delta to
+   * `enc_count_`.
+   *
+   * @param arg Pointer to the `Encoder` instance (registered via `set_interrupt`).
+   */
  static IRAM_ATTR void encoder_isr(void *arg);

  Common::GPIO pin_a_;
  Common::GPIO pin_b_;
  uint32_t cpr_;

  volatile uint8_t enc_val_ = 0;
  std::atomic<int32_t> enc_count_{0};
};

} // namespace SENSORS
