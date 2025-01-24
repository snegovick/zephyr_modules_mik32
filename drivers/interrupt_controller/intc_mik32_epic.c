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

#include <zephyr/arch/riscv/irq.h>
#include <zephyr/arch/riscv/csr.h>

#include <zephyr/soc/mik32_memory_map.h>
#include <zephyr/soc/mik32_irq.h>

#include <soc/mikron/mik32/soc.h>

#include <zephyr/sw_isr_table.h>

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

struct _isr_table_entry _sw_isr_table[32] = {
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
	{.arg = NULL, .isr = z_irq_spurious},
};

static int epic_init(void)
{
	//printk("epic init\n");
	/* for (int i = 0; i < 32; i ++) { */
	/* 	_sw_isr_table[i].isr = &z_irq_spurious; */
	/* 	_sw_isr_table[i].arg = NULL; */
	/* } */
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
		uint32_t status = EPIC->STATUS;
		clear_csr(mip, MIP_MEIP);
		for (int i = 0; i < 32; i ++) {
			if (status & (1 << i)) {
				_sw_isr_table[i].isr(_sw_isr_table[i].arg);
			}
		}
		EPIC->CLEAR = 0xfffffffful;
	}
}

SYS_INIT(epic_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);
