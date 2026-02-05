idf_component_register(
    SRCS "src/ble.cpp"
    INCLUDE_DIRS "include"
    REQUIRES espp__ble_gatt_server espp__cli imu
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra
)
