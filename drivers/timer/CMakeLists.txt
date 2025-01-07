# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RISCV_SCR1_MACHINE_TIMER riscv_scr1_machine_timer.c)
