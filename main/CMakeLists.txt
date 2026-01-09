idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES log
)

# Stricter compiler flags for better C++
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++20
    -Wall
    -Wextra
)
