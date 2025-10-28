idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES icm20948 force_sensor bluetooth_manager sensor_fusion common log
)

# Stricter compiler flags for better C++
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++17
    -Wall
    -Wextra
    
)
