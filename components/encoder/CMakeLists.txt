idf_component_register(
    SRCS "src/encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common 
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++20
    -Wall
    -Wextra
)
