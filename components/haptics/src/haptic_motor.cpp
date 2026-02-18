/*
 * haptic_motor.cpp
 *  - DRV2605L Haptic Motor Driver for ESP32
 *  - Adapted from SparkFun DRV2605L Arduino Library by Mary West
 *  - Modified to use espp::I2c for ESP32-S3
 */

#include "haptic_motor.hpp"

using namespace HAPTICS;

DRV2605L::DRV2605L(espp::I2c &i2c, uint8_t address) : i2c_(i2c), address_(address) {}

/*
 * Initialize the DRV2605L device
 * Returns true if device is detected and ready
 */
bool DRV2605L::init() {
  uint8_t status = get_status();

  // Check if device is present and not in over-current protection
  // Expected value is 0xE0 (device ID = 7, no errors)
  if ((status & 0xE0) != EXPECTED_STATUS)
    return false;

  // default inits
  // Set to internal trigger mode
  set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);

  // Select ERM motor
  select_motor(HAPTICS::DRV2605L::MotorType::ERM);

  // Select library
  select_library(HAPTICS::DRV2605L::Library::ERM_LIB_A);

  return true;
}

/*
 * Check if device is ready (not in standby, no errors)
 */
bool DRV2605L::is_device_ready() {
  uint8_t status = get_status();
  // Check device ID (bits 7-5) and no error flags (bits 3,1,0)
  return (status & 0xE0) == EXPECTED_STATUS && (status & 0x0B) == 0;
}

/*
 * Read the status register
 */
uint8_t DRV2605L::get_status() {
  return read_register(REGISTERS::STATUS);
}

/*
 * Set the operation mode
 */
void DRV2605L::set_mode(Mode mode) {
  write_register(REGISTERS::MODE, static_cast<uint8_t>(mode));
}

/*
 * Select motor type (ERM or LRA)
 */
void DRV2605L::select_motor(MotorType motor) {
  write_register(REGISTERS::FEEDBACK, static_cast<uint8_t>(motor));
}

/*
 * Select waveform library
 */
void DRV2605L::select_library(Library lib) {
  write_register(REGISTERS::LIBRARY_SEL, static_cast<uint8_t>(lib));
}

/*
 * Set waveform for a specific sequencer
 * sequencer: 0-7 (sequencer number)
 * waveform: waveform ID from the selected library
 */
void DRV2605L::set_waveform(uint8_t sequencer, uint8_t waveform) {
  if (sequencer < 8) {
    write_register(REGISTERS::WAVESEQ1 + sequencer, waveform);
  }
}

/*
 * Trigger the waveform playback
 */
void DRV2605L::go() {
  write_register(REGISTERS::GO, 0x01);
}

/*
 * Stop waveform playback
 */
void DRV2605L::stop() {
  write_register(REGISTERS::GO, 0x00);
}

/*
 * Set real-time playback value
 * Mode must be set to REALTIME_PLAYBACK
 */
void DRV2605L::set_realtime_value(uint8_t value) {
  write_register(REGISTERS::RTP, value);
}

/*
 * Set overdrive time offset
 */
void DRV2605L::set_overdrive(uint8_t value) {
  write_register(REGISTERS::OVERDRIVE, value);
}

/*
 * Set sustain time offset (positive)
 */
void DRV2605L::set_sustain_pos(uint8_t value) {
  write_register(REGISTERS::SUSTAIN_OFFSET_POS, value);
}

/*
 * Set sustain time offset (negative)
 */
void DRV2605L::set_sustain_neg(uint8_t value) {
  write_register(REGISTERS::SUSTAIN_OFFSET_NEG, value);
}

/*
 * Set brake time offset
 */
void DRV2605L::set_break_time(uint8_t value) {
  write_register(REGISTERS::BREAKTIME, value);
}

/*
 * Set audio-to-vibe control
 */
void DRV2605L::set_audio_to_vibe(uint8_t value) {
  write_register(REGISTERS::AUDIO_CTRL, value);
}

/*
 * Set audio-to-vibe minimum input level
 */
void DRV2605L::set_audio_min_level(uint8_t value) {
  write_register(REGISTERS::AUDIO_MIN_LVL, value);
}

/*
 * Set audio-to-vibe maximum input level
 */
void DRV2605L::set_audio_max_level(uint8_t value) {
  write_register(REGISTERS::AUDIO_MAX_LVL, value);
}

/*
 * Set audio-to-vibe minimum output drive
 */
void DRV2605L::set_audio_min_drive(uint8_t value) {
  write_register(REGISTERS::AUDIO_MIN_DRIVE, value);
}

/*
 * Set audio-to-vibe maximum output drive
 */
void DRV2605L::set_audio_max_drive(uint8_t value) {
  write_register(REGISTERS::AUDIO_MAX_DRIVE, value);
}

/*
 * Set rated voltage for the motor
 */
void DRV2605L::set_rated_voltage(uint8_t value) {
  write_register(REGISTERS::RATED_VOLT, value);
}

/*
 * Set overdrive clamp voltage
 */
void DRV2605L::set_overdrive_clamp(uint8_t value) {
  write_register(REGISTERS::OVERDRIVE_CLAMP, value);
}

/*
 * Set control register 1 (AC coupling, drive time)
 */
void DRV2605L::set_control1(uint8_t value) {
  write_register(REGISTERS::CONTROL1, value);
}

/*
 * Set control register 2
 */
void DRV2605L::set_control2(uint8_t value) {
  write_register(REGISTERS::CONTROL2, value);
}

/*
 * Set control register 3
 */
void DRV2605L::set_control3(uint8_t value) {
  write_register(REGISTERS::CONTROL3, value);
}

/*
 * Set control register 4
 */
void DRV2605L::set_control4(uint8_t value) {
  write_register(REGISTERS::CONTROL4, value);
}

/*
 * Set control register 5
 */
void DRV2605L::set_control5(uint8_t value) {
  write_register(REGISTERS::CONTROL5, value);
}

/*
 * Set LRA open loop period
 */
void DRV2605L::set_open_loop_period(uint8_t value) {
  write_register(REGISTERS::OLP, value);
}

/*
 * Get battery voltage
 * Formula: Vdd = Vbatt[7:0] x 5.6V / 255
 */
float DRV2605L::get_battery_voltage() {
  uint8_t vbatt = read_register(REGISTERS::VBAT_MONITOR);
  return (static_cast<float>(vbatt) * 5.6f) / 255.0f;
}

/*
 * Get LRA resonance period in microseconds
 * Formula: LRA Period(us) = LRA_Period[7:0] x 98.46us
 */
float DRV2605L::get_lra_resonance_period_us() {
  uint8_t period = read_register(REGISTERS::LRA_RES_PERIOD);
  return static_cast<float>(period) * 98.46f;
}

/*
 * Get auto-calibration compensation result
 */
uint8_t DRV2605L::get_comp_result() {
  return read_register(REGISTERS::COMP_RESULT);
}

/*
 * Get auto-calibration back-EMF result
 */
uint8_t DRV2605L::get_backemf_result() {
  return read_register(REGISTERS::BACKEMF);
}

/*
 * Write a value to a register
 */
void DRV2605L::write_register(uint8_t reg, uint8_t value) {
  uint8_t data[2] = {reg, value};
  i2c_.write(address_, data, sizeof(data));
}

/*
 * Read a value from a register
 */
uint8_t DRV2605L::read_register(uint8_t reg) {
  uint8_t value = 0;
  i2c_.read_at_register(address_, reg, &value, sizeof(value));
  return value;
}
