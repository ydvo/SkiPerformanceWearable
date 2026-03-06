# FSR (Force‑Sensitive Resistor) Component

**Purpose**
Wraps an ADC channel to read the voltage across a force‑sensitive resistor, providing calibrated pressure (mV) values for lean‑angle detection.

**Public API (class `SENSORS::fsr`)**
- Constructor `fsr(adc1_channel_t channel, gpio_num_t enable_pin = GPIO_NUM_NC)` – store ADC channel; optional GPIO to power the sensor.
- `esp_err_t init()` – configure ADC1 for the selected channel; optionally set enable pin as output and drive high.
- `float read_mv()` – perform a single ADC conversion, convert raw counts to millivolts using calibrated scale factor.

**Thread‑Safety**
`read_mv()` can be called from any task after `init()`; ADC driver is thread‑safe in ESP‑IDF.

**Typical Usage**
```cpp
SENSORS::fsr pressure_sensor(ADC1_CHANNEL_3);
pressure_sensor.init();
float pressure_mv = pressure_sensor.read_mv();
```