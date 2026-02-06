idf_component_register(
    SRCS "src/fsr.cpp"
    INCLUDE_DIRS "include"
    REQUIRES espp__adc
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra
)
