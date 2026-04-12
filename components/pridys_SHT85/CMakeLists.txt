idf_component_register(
    SRCS "./src/pridys_SHT85.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c
)
