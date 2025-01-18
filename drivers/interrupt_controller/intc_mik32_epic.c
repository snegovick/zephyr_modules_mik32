/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_epic

#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <hal/mik32/shared/include/mik32_memory_map.h>
#include <hal/mik32/shared/periphery/epic.h>
#include <hal/mik32/shared/periphery/scr1_timer.h>
#include <zephyr/arch/riscv/irq.h>

#define MIE_MEIE                    (0x1 << 11)
#define MIP_MTIP                    (0x1 << 7)
#define MIP_MEIP                    (0x1 << 11)

void arch_irq_enable(unsigned int irq)
{
	EPIC->MASK_LEVEL_SET = 1<<irq;
}

void arch_irq_disable(unsigned int irq)
{
	EPIC->MASK_LEVEL_CLEAR = 1<<irq;
}

int arch_irq_is_enabled(unsigned int irq)
{
	if (EPIC->MASK_LEVEL_SET & (1 << irq)) {
		return 1;
	}
	return 0;
}

static int epic_init(void)
{
	printk("epic init\n");
	set_csr(mstatus, MSTATUS_MIE);
	set_csr(mie, MIE_MEIE);
	return 0;
}

void scr1_timer_isr();

void __soc_handle_all_irqs(void) {
	unsigned long cause = read_csr(mcause);
	if ( ((cause & CONFIG_RISCV_MCAUSE_EXCEPTION_MASK) == 7) && (cause & (RISCV_MCAUSE_IRQ_BIT)) ) {
		clear_csr(mip, MIP_MTIP);
		scr1_timer_isr();
	} else {
		clear_csr(mip, MIP_MEIP);
		EPIC->CLEAR = 0xfffffffful;
	}
}

SYS_INIT(epic_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);
