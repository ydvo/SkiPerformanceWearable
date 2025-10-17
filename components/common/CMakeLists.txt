idf_component_register(
    SRCS "src/*"
    INCLUDE_DIRS "include"
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++17 -Wall -Wextra -Werror -Wpedantic
)
