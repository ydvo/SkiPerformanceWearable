idf_component_register(
  SRCS "src/GPIO.cpp"
  INCLUDE_DIRS "include"
  REQUIRES  esp_driver_gpio
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra  
)
