# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_MIK32_GPIO_IRQ               intc_mik32_gpio_irq.c)
zephyr_library_sources_ifdef(CONFIG_MIK32_EPIC               intc_mik32_epic.c)
