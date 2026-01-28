idf_component_register(
  SRCS "src/powerstats.cpp" "src/fuelgauge.cpp"
  INCLUDE_DIRS "include"
  REQUIRES espp__i2c
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra  
)
