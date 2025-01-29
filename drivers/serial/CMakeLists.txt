# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/uart.h)

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_MIK32_USART usart_mik32.c)
