idf_component_register(
    SRCS 
        "./src/pridys_SHT85.c"
        "./i2c_commands/src/pridys_SHT85_i2c_commands.c"
        "./crc_handler/src/pridys_SHT85_crc_handler.c"
        "./status_register/src/pridys_SHT85_status_register.c"
    INCLUDE_DIRS 
        "."
        "./i2c_commands"
        "./crc_handler"
        "./status_register"
    REQUIRES esp_driver_i2c
)
