idf_component_register(
    SRCS "mfcc.c" "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2s esp_driver_gpio
    PRIV_REQUIRES esp-dsp esp-nimble-cpp
)