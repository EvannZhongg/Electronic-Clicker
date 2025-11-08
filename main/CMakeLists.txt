idf_component_register(SRCS "blink_example_main.c"
                      INCLUDE_DIRS "."

                      PRIV_REQUIRES bt nvs_flash log esp_driver_gpio freertos
                    )