# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

if(CONFIG_SOC_MIK32)

zephyr_include_directories(mik32/peripherals/Include)

zephyr_library_sources(mik32/peripherals/Source/mik32_hal.c)

zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_ADC     mik32/peripherals/Source/mik32_hal_adc.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_DAC     mik32/peripherals/Source/mik32_hal_dac.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_DMA     mik32/peripherals/Source/mik32_hal_dma.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_GPIO    mik32/peripherals/Source/mik32_hal_gpio.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_I2C     mik32/peripherals/Source/mik32_hal_i2c.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_HAL_UART    mik32/peripherals/Source/mik32_hal_uart.c)

endif
