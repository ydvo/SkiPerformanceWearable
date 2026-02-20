/* encoder.cpp
 *  - Quadrature rotary encoder driver using lookup table decoding
 */

#include "encoder.hpp"

#include <cmath>

#include "esp_attr.h"
#include "esp_log.h"

namespace SENSORS {

static const char *TAG = "Encoder";

// Lookup table for quadrature decoding
// Indexed by (prev_AB << 2 | curr_AB), yields direction:
//   +1 = CW step, -1 = CCW step, 0 = no movement or invalid
static const int8_t DRAM_ATTR lookup_table[] = {
    0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0,
};

Encoder::Encoder(gpio_num_t pin_a, gpio_num_t pin_b, uint32_t cpr)
    : pin_a_(pin_a, Common::GPIO::INPUT, false, Common::GPIO::PULLUP),
      pin_b_(pin_b, Common::GPIO::INPUT, false, Common::GPIO::PULLUP), cpr_(cpr) {}

esp_err_t Encoder::init() {
  ESP_LOGI(TAG, "Initializing encoder on GPIO %d (A) and GPIO %d (B), CPR=%lu",
           pin_a_.getPin(), pin_b_.getPin(), (unsigned long)cpr_);

  // Seed enc_val_ with current pin state so first interrupt has valid history
  uint8_t a = pin_a_.getLevel() ? 1 : 0;
  uint8_t b = pin_b_.getLevel() ? 1 : 0;
  enc_val_ = (a << 1) | b;

  // Attach any-edge interrupt on channel A
  esp_err_t ret =
      pin_a_.set_interrupt(Common::GPIO::INTERRUPT_ANY_EDGE, encoder_isr, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to attach interrupt on channel A");
    return ret;
  }

  // Attach any-edge interrupt on channel B
  ret = pin_b_.set_interrupt(Common::GPIO::INTERRUPT_ANY_EDGE, encoder_isr, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to attach interrupt on channel B");
    return ret;
  }

  ESP_LOGI(TAG, "Encoder initialized");
  return ESP_OK;
}

int32_t Encoder::get_count() const { return enc_count_.load(std::memory_order_relaxed); }

float Encoder::get_angle() const {
  int32_t count = enc_count_.load(std::memory_order_relaxed);
  float angle = fmodf((count * 360.0f / cpr_), 360.0f);
  if (angle < 0.0f) {
    angle += 360.0f;
  }
  return angle;
}

void Encoder::reset() { enc_count_.store(0, std::memory_order_relaxed); }

IRAM_ATTR void Encoder::encoder_isr(void *arg) {
  Encoder *self = static_cast<Encoder *>(arg);

  // Shift previous state left and read current pin levels
  self->enc_val_ <<= 2;
  uint8_t a = gpio_get_level(self->pin_a_.getPin()) ? 1 : 0;
  uint8_t b = gpio_get_level(self->pin_b_.getPin()) ? 1 : 0;
  self->enc_val_ |= (a << 1) | b;

  int8_t delta = lookup_table[self->enc_val_ & 0x0F];
  if (delta != 0) {
    self->enc_count_.fetch_add(delta, std::memory_order_relaxed);
  }
}

} // namespace SENSORS
