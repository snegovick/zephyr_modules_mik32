/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__initialize)
#if defined(CONFIG_GEN_SW_ISR_TABLE)
GTEXT(_isr_wrapper)
#endif

SECTION_FUNC(vectors, __start)
#if defined(CONFIG_RISCV_JUMP_TO_CODE)
	auipc t0, CONFIG_RISCV_JUMP_TO_CODE_OFFSET
	jalr t0, 8
	nop
	nop
#endif
#if defined(CONFIG_RISCV_GP)
	/* Initialize global pointer */
	.option push
	.option norelax
	la gp, __global_pointer$
	.option pop
#endif

	.option norvc;

#if defined(CONFIG_RISCV_USE_START_AS_MTVEC)
	la t0, __start
#endif
	csrw mtvec, t0

	/* Jump to __reset */
	tail __reset
