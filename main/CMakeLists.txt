# Both transport drivers are listed unconditionally: component requirements are expanded
# before Kconfig is evaluated, so they cannot be gated on CONFIG_TARGET_TEST_INTERFACE_*.
# Unused transport code is compiled out via #if in target-test.c.
idf_component_register(SRCS "target-test.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_spi esp_driver_i2c esp_driver_uart esp_driver_gpio)
