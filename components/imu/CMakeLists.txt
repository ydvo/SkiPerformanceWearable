idf_component_register(
    SRCS "src/imu.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common icm20948 filters
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    
)
