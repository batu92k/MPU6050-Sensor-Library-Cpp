set(PROJ_ROOT "../../../../")

idf_component_register(SRCS "example_esp32.cpp"
                            "${PROJ_ROOT}examples/ESP32/_common/esp32_i2c_if.cpp"
                            "${PROJ_ROOT}i2c_interface.cpp"
                            "${PROJ_ROOT}mpu6050.cpp"
                    INCLUDE_DIRS "${PROJ_ROOT}"
                                "${PROJ_ROOT}examples/ESP32/_common/")
