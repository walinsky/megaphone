set(srcs)

idf_component_register(SRCS "codec.c" "main.c" "${srcs}"
                    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio
                    INCLUDE_DIRS ".")
