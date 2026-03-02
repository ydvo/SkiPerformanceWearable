/**
 * @file GPIO.hpp
 * @ingroup utilities
 * @brief GPIO wrapper for ESP-IDF.
 *
 * Provides a lightweight C++ class to configure and control a single GPIO pin,
 * including direction, level, pull‑up/down, and optional interrupt handling.
 */

#pragma once
#include "driver/gpio.h"
#include "esp_err.h"

namespace Common {

/**
 * @brief Simple GPIO wrapper class.
 *
 * Manages a single GPIO pin's configuration and runtime state. The class is
 * intended for use from any FreeRTOS task; ISR registration is performed via
 * `set_interrupt()` which installs a GPIO ISR service (once per system) and
 * attaches the provided handler.
 */
class GPIO {
public:
  /**
 * @brief GPIO direction mode.
 *
 * Mirrors ESP-IDF `gpio_mode_t` values for input, output, or bidirectional
 * operation.
 */
enum Direction {
    INPUT = GPIO_MODE_INPUT,
    OUTPUT = GPIO_MODE_OUTPUT,
    INPUT_OUTPUT = GPIO_MODE_INPUT_OUTPUT
  };
  /**
 * @brief Logical output level for GPIO.
 *
 * Used with `setLevel()` and `getLevel()`. Values map to low (0) and high (1).
 */
enum Level {
    ON = 1,
    OFF = 0,
  };
  /**
 * @brief Pull‑up/down configuration.
 *
 * Selects no pull, internal pull‑up, or internal pull‑down resistor.
 */
enum Pull {
    NONE = 0,
    PULLUP = GPIO_PULLUP_ONLY,
    PULLDOWN = GPIO_PULLDOWN_ONLY,
  };
  /**
 * @brief GPIO interrupt trigger type.
 *
 * Mirrors ESP‑IDF interrupt configurations for edge detection.
 */
enum InterruptType {
    INTERRUPT_NONE = GPIO_INTR_DISABLE,
    INTERRUPT_RISING_EDGE = GPIO_INTR_POSEDGE,
    INTERRUPT_FALLING_EDGE = GPIO_INTR_NEGEDGE,
    INTERRUPT_ANY_EDGE = GPIO_INTR_ANYEDGE
  };

  /**
   * @brief Construct a GPIO wrapper.
   *
   * Configures the specified pin with the given direction, optional initial
   * output level, and pull‑up/down setting.
   *
   * @param pin            GPIO number to manage.
   * @param dir            Desired pin direction (default INPUT).
   * @param initial_level  Initial output level when `dir` includes OUTPUT.
   * @param pull           Pull‑up/pull‑down configuration (default NONE).
   */
  explicit GPIO(gpio_num_t pin, Direction dir = INPUT, bool initial_level = false,
                 Pull pull = NONE);
  /**
   * @brief Destroy the GPIO object.
   *
   * If an interrupt was registered, it is removed from the ISR service.
   */
  ~GPIO();

  /**
   * @brief Set the GPIO output level.
+   *
+   * @param level Desired level (true = high, false = low).
+   * @return ESP_OK on success or an ESP‑IDF error code.
+   */
  esp_err_t setLevel(bool level);

  /**
   * @brief Read the current GPIO level.
+   *
+   * @return Current level (true = high, false = low).
+   */
  bool getLevel() const;

  /**
   * @brief Retrieve the GPIO number managed by this instance.
+   *
+   * @return GPIO number identifier.
+   */
  gpio_num_t getPin() const;

  /**
   * @brief Enable and register a GPIO interrupt.
+   *
+   * Installs the global GPIO ISR service if not already installed, sets the
+   * interrupt trigger type, and attaches the supplied handler. The optional
+   * `arg` is passed to the handler; if omitted the `GPIO` instance pointer is
+   * used.
+   *
+   * @param type  Interrupt trigger type (rising, falling, etc.).
+   * @param handler ISR callback conforming to `gpio_isr_t` signature.
+   * @param arg   Optional user data passed to the handler (default nullptr).
+   * @return ESP_OK on success or an ESP‑IDF error code.
+   */
  esp_err_t set_interrupt(InterruptType type, gpio_isr_t handler, void *arg = nullptr);

  /**
   * @brief Disable and detach a previously registered GPIO interrupt.
+   *
+   * Removes the ISR handler for the pin and disables interrupt generation.
+   *
+   * @return ESP_OK on success or an ESP‑IDF error code.
+   */
  esp_err_t disable_interrupt();

private:
  gpio_num_t pin_;
  bool interrupt_enabled_ = false;
};

} // namespace Common
