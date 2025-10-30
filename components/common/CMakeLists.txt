idf_component_register(
  SRCS "src/I2C.cpp" "src/GPIO.cpp"
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_i2c esp_driver_gpio
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++17 -Wall -Wextra  
)
