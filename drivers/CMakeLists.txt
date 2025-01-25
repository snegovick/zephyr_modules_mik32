# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

add_subdirectory(interrupt_controller)

add_subdirectory_ifdef(CONFIG_EEPROM eeprom)
add_subdirectory_ifdef(CONFIG_MEMC memc)
add_subdirectory_ifdef(CONFIG_SYS_CLOCK_EXISTS timer)
add_subdirectory_ifdef(CONFIG_GPIO gpio)
add_subdirectory_ifdef(CONFIG_I2C i2c)
add_subdirectory_ifdef(CONFIG_CLOCK_CONTROL clock_control)
add_subdirectory_ifdef(CONFIG_SERIAL serial)
add_subdirectory_ifdef(CONFIG_PINCTRL pinctrl)
