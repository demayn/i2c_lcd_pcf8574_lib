idf_component_register(SRCS "i2c_lcd_pcf8574.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver")

# Get the latest Git tag
execute_process(
    COMMAND git describe --tags --abbrev=0
    OUTPUT_VARIABLE APP_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Pass the version to the compiler
add_definitions(-DAPP_VERSION=\"${APP_VERSION}\")
