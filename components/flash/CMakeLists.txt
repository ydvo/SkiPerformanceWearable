idf_component_register(
    SRCS "src/flash.cpp"
    INCLUDE_DIRS "include"
    REQUIRES common spi_flash espp__logger
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
)