idf_component_register(
    SRCS "src/haptic_motor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES espp__i2c
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra
)
