idf_component_register(
    SRCS "src/encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common esp_driver_gpio
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++20
    -Wall
    -Wextra
)
