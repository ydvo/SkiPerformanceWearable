idf_component_register(
    SRCS "src/force_sensor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra  -Wpedantic
)
