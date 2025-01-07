# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_UART_MIK32_USART usart_mik32.c)
