/*
 * haptic_motor.hpp
 *  - DRV2605L Haptic Motor Driver for ESP32
 *  - Adapted from SparkFun DRV2605L Arduino Library by Mary West
 *  - Modified to use espp::I2c for ESP32-S3
 */

#pragma once
#include "i2c.hpp"
#include <cstdint>

namespace HAPTICS {

class DRV2605L {
public:
  enum class Mode : uint8_t {
    INTERNAL_TRIGGER = 0x00,
    EXTERNAL_TRIGGER_EDGE = 0x01,
    EXTERNAL_TRIGGER_LEVEL = 0x02,
    PWM_ANALOG = 0x03,
    AUDIO_TO_VIBE = 0x04,
    REALTIME_PLAYBACK = 0x05,
    DIAGNOSTICS = 0x06,
    AUTO_CALIBRATION = 0x07
  };

  enum class MotorType : uint8_t {
    ERM = 0x39, // ERM Mode, 4x Brake factor, Medium Gain, 1.365x Back EMF
    LRA = 0xB9  // LRA MODE, 4X Brake Factor, Medium Gain, 7.5x Back EMF
  };

  enum class Library : uint8_t {
    EMPTY = 0x00,
    ERM_LIB_A = 0x01,
    ERM_LIB_B = 0x02,
    ERM_LIB_C = 0x03,
    ERM_LIB_D = 0x04,
    ERM_LIB_E = 0x05,
    LRA_LIB = 0x06
  };

  enum REGISTERS : uint8_t {
    STATUS = 0x00,
    MODE = 0x01,
    RTP = 0x02,
    LIBRARY_SEL = 0x03,
    WAVESEQ1 = 0x04,
    WAVESEQ2 = 0x05,
    WAVESEQ3 = 0x06,
    WAVESEQ4 = 0x07,
    WAVESEQ5 = 0x08,
    WAVESEQ6 = 0x09,
    WAVESEQ7 = 0x0A,
    WAVESEQ8 = 0x0B,
    GO = 0x0C,
    OVERDRIVE = 0x0D,
    SUSTAIN_OFFSET_POS = 0x0E,
    SUSTAIN_OFFSET_NEG = 0x0F,
    BREAKTIME = 0x10,
    AUDIO_CTRL = 0x11,
    AUDIO_MIN_LVL = 0x12,
    AUDIO_MAX_LVL = 0x13,
    AUDIO_MIN_DRIVE = 0x14,
    AUDIO_MAX_DRIVE = 0x15,
    RATED_VOLT = 0x16,
    OVERDRIVE_CLAMP = 0x17,
    COMP_RESULT = 0x18,
    BACKEMF = 0x19,
    FEEDBACK = 0x1A,
    CONTROL1 = 0x1B,
    CONTROL2 = 0x1C,
    CONTROL3 = 0x1D,
    CONTROL4 = 0x1E,
    CONTROL5 = 0x1F,
    OLP = 0x20,
    VBAT_MONITOR = 0x21,
    LRA_RES_PERIOD = 0x22
  };

  // Effect IDs from the DRV2605L ROM library (ERM Library A/B/C, see datasheet Table 11)
  enum EFFECTS : uint8_t {
    // Clicks
    SINGLE_CLICK         = 1,
    DOUBLE_CLICK         = 14,
    TRIPLE_CLICK         = 16,
    SHARP_CLICK          = 47,
    SHARP_CLICK_2        = 48,
    SHARP_CLICK_3        = 49,
    SOFT_BUMP            = 52,
    SOFT_BUMP_2          = 53,
    SOFT_BUMP_3          = 54,

    // Pulses / alerts
    STRONG_BUZZ          = 62,
    LONG_BUZZ            = 70,
    ALERT_750MS          = 71,
    ALERT_1000MS         = 72,
    SHORT_DOUBLE_CLICK   = 11,
    SHORT_DOUBLE_CLICK_2 = 12,

    // Transitions / confirmations
    RAMP_UP              = 58,
    RAMP_DOWN            = 59,
    TRANSITION_CLICK     = 65,
    TRANSITION_HUM       = 66,

    // Sequence terminator (do not play)
    END                  = 0,
  };

  /**
   * Program the sequencer with up to 8 effects and fire go().
   * The array must be terminated with EFFECTS::END or have exactly `count` entries.
   * At most 8 slots are used; extra entries are ignored.
   * All 8 sequence registers + GO are written in a single I2C transaction.
   *
   * Example:
   *   uint8_t seq[] = { EFFECTS::SINGLE_CLICK, EFFECTS::END };
   *   haptic.play(seq, 2);
   *
   * @return true on success, false if the I2C write failed
   */
  bool play(const uint8_t *effects, uint8_t count);

  static constexpr uint8_t DEFAULT_ADDRESS{0x5A};
  static constexpr uint8_t EXPECTED_STATUS{0xE0};

  explicit DRV2605L(espp::I2c &i2c, uint8_t address = DEFAULT_ADDRESS);

  bool init();
  bool is_device_ready();
  uint8_t get_status();

  void set_mode(Mode mode);
  void select_motor(MotorType motor);
  void select_library(Library lib);
  void set_waveform(uint8_t sequencer, uint8_t waveform);
  bool go();
  void stop();

  void set_realtime_value(uint8_t value);

  void set_overdrive(uint8_t value);
  void set_sustain_pos(uint8_t value);
  void set_sustain_neg(uint8_t value);
  void set_break_time(uint8_t value);

  void set_audio_to_vibe(uint8_t value);
  void set_audio_min_level(uint8_t value);
  void set_audio_max_level(uint8_t value);
  void set_audio_min_drive(uint8_t value);
  void set_audio_max_drive(uint8_t value);

  void set_rated_voltage(uint8_t value);
  void set_overdrive_clamp(uint8_t value);

  void set_control1(uint8_t value);
  void set_control2(uint8_t value);
  void set_control3(uint8_t value);
  void set_control4(uint8_t value);
  void set_control5(uint8_t value);

  void set_open_loop_period(uint8_t value);

  float get_battery_voltage();
  float get_lra_resonance_period_us();

  uint8_t get_comp_result();
  uint8_t get_backemf_result();

private:
  espp::I2c &i2c_;
  uint8_t address_;

  bool write_register(uint8_t reg, uint8_t value);
  uint8_t read_register(uint8_t reg);
};

} // namespace HAPTICS
