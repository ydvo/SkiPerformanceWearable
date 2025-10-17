idf_component_register(
    SRCS "src/sensor_fusion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common lcm20948 force_sensor
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++17 -Wall -Wextra -Werror -Wpedantic
)
