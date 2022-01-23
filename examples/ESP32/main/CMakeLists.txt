idf_component_register(SRCS "example_esp32.cpp"
                            "esp32_i2c_if.cpp"
                            "../../../i2c_interface.cpp"
                            "../../../mpu6050.cpp"
                    INCLUDE_DIRS "../../../")
