/**
 * @file fsr.hpp
 * @ingroup sensors
 * @brief Force‑sensitive resistor (FSR) driver.
 *
 * Provides a thin wrapper around the ESP‑IDF oneshot ADC driver to read a
 * resistive pressure sensor. Handles calibration of baseline (no pressure) and
 * maximum pressure, threshold detection, and conversion to a percentage.
 */
#pragma once
#include "adc_types.hpp"
#include "hal/adc_types.h"
#include "oneshot_adc.hpp"

namespace SENSORS {

/* Constants */

/**
 * @brief Simple FSR driver.
 *
 * Wraps an ESP‑IDF oneshot ADC to read voltage from a force‑sensitive
 * resistor. Calibration methods compute baseline and maximum voltages; a
 * configurable pressure threshold enables `is_pressed()` detection. The class is
 * intended for use from a single FreeRTOS task; concurrent access requires
 * external synchronization.
 */
class fsr {

public:
  /**
   * @brief Constructor for FSR sensor
   * @param channel The ADC channel the FSR is connected to
   */
  fsr(adc_channel_t channel);
  /**
   * @brief Destructor; releases any allocated resources.
+   *
+   * Currently no dynamic resources are allocated, but the destructor is
+   * provided for completeness and future extensions.
+   */
  ~fsr();

  /**
   * @brief Read the raw ADC value
   * @return Raw ADC value (0-4095 for 12-bit ADC)
   */
  /**
   * @brief Read the raw ADC code (units of LSB).
+   *
+   * @return Raw ADC value (e.g., 0‑4095 for a 12‑bit ADC). Returns 0 on error.
+   */
  int read_raw();

  /**
   * @brief Read the voltage in millivolts
   * @return Voltage in mV (with attenuation, range is ~0-3300mV)
   */
  float read();

  /**
   * @brief Set a pressure threshold in millivolts
   * @param threshold_mv Threshold voltage in mV. When read() voltage exceeds
   * this, is_pressed() will return true
   */
  void set_pressure_threshold(float threshold_mv);

  /**
   * @brief Get the current pressure threshold
   * @return Current threshold in mV
   */
  float get_pressure_threshold() const {
    return pressure_threshold_;
  }

  /**
   * @brief Check if the FSR is currently pressed (above threshold)
   * @return true if the last read() exceeded the threshold, false otherwise
   */
  bool is_pressed() const {
    return is_pressed_;
  }

  /**
   * @brief Check if pressure exceeds a given threshold
   * @param threshold_mv Threshold to check against in mV
   * @return true if current reading >= threshold, false otherwise
   */
  /**
   * @brief Test the current pressure against a specified threshold.
+   *
+   * Reads the current voltage and compares it to `threshold_mv`.
+   *
+   * @param threshold_mv Threshold voltage in millivolts.
+   * @return true if the measured voltage >= threshold_mv, false otherwise.
+   */
  bool check_pressure(float threshold_mv);

  /**
   * @brief Calibrate the baseline (no pressure) value
   * Takes 10 samples and averages them. Keep no pressure on sensor during this.
   */
  void calibrate_baseline();

  /**
   * @brief Calibrate the maximum pressure value
   * Takes 10 samples and averages them. Apply full pressure during this.
   */
  void calibrate_max();

  /**
   * @brief Manually set calibration values
   * @param baseline_mv Baseline voltage in mV (no pressure)
   * @param max_mv Maximum voltage in mV (full pressure)
   */
  void set_calibration(float baseline_mv, float max_mv);

  /**
   * @brief Get the calibrated baseline value
   * @return Baseline voltage in mV
   */
  float get_baseline() const {
    return baseline_mv_;
  }

  /**
   * @brief Get the calibrated maximum value
   * @return Maximum voltage in mV
   */
  float get_max() const {
    return max_mv_;
  }

  /**
   * @brief Read pressure as a percentage (0-100%)
   * Uses calibration values to convert voltage to percentage
   * @return Pressure as percentage (0-100)
   */
  float read_percentage();

private:
  std::vector<espp::AdcConfig> channels_; ///< ADC channel configuration vector.
  espp::OneshotAdc adc_;                  ///< Oneshot ADC instance used for reads.
  float pressure_threshold_;              ///< Pressure threshold in millivolts; 0 disables detection.
  bool is_pressed_;                       ///< Cached flag set when last reading exceeded the threshold.
  float baseline_mv_;                     ///< Calibrated baseline voltage (no pressure) in mV.
  float max_mv_;                          ///< Calibrated maximum voltage (full pressure) in mV.
};
} // namespace SENSORS
