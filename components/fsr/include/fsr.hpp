/* fsr.hpp
 *  simple driver for force resistive sensor. basically a wrapper of the adc
 */
#pragma once
#include "adc_types.hpp"
#include "hal/adc_types.h"
#include "oneshot_adc.hpp"

namespace SENSOR {

/* Constants */

class fsr {

public:
  /**
   * @brief Constructor for FSR sensor
   * @param channel The ADC channel the FSR is connected to
   */
  fsr(adc_channel_t channel);
  ~fsr();

  /**
   * @brief Read the raw ADC value
   * @return Raw ADC value (0-4095 for 12-bit ADC)
   */
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
  std::vector<espp::AdcConfig> channels_; // channels to read from
  espp::OneshotAdc adc_;                  // ADC instance
  float pressure_threshold_;              // pressure threshold in mV
  bool is_pressed_;                       // flag indicating if pressed
  float baseline_mv_;                     // baseline voltage (no pressure)
  float max_mv_;                          // max voltage (full pressure)
};
} // namespace SENSOR
