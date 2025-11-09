idf_component_register(
    SRCS "src/sensor_fusion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common imu force_sensor
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra  
)
