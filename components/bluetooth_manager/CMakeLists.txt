idf_component_register(
    SRCS "src/bluetooth_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash bt common
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -std=gnu++17 -Wall -Wextra  
)
