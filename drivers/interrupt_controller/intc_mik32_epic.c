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
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mik32.h>
#include <zephyr/dt-bindings/clock/mik32-clocks.h>

#include <soc/mikron/mik32/soc.h>

#include <zephyr/sw_isr_table.h>

void arch_irq_enable(unsigned int irq)
{
	EPIC->MASK_EDGE_SET = 1<<irq;
}

void arch_irq_disable(unsigned int irq)
{
	EPIC->MASK_EDGE_CLEAR = 1<<irq;
}

int arch_irq_is_enabled(unsigned int irq)
{
	if (EPIC->MASK_EDGE_SET & (1 << irq)) {
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

int mik32_irq_connect_dynamic(unsigned int	irq, unsigned int	priority, void(* routine )(const void *dev), const void * parameter, uint32_t	flags)
{
  _sw_isr_table[irq].isr = routine;
  _sw_isr_table[irq].arg = parameter;
  return 0;
}

int mik32_irq_disconnect_dynamic (unsigned int irq)
{
  _sw_isr_table[irq].isr = z_irq_spurious;
  _sw_isr_table[irq].arg = NULL;
  return 0;
}

static int epic_init(void)
{
  uint16_t clkid = MIK32_CLOCK_EPIC;
	(void)clock_control_on(MIK32_CLOCK_CONTROLLER, (clock_control_subsys_t)&clkid);
	set_csr(mstatus, MSTATUS_MIE);
	set_csr(mie, MIE_MEIE);
  EPIC->MASK_EDGE_CLEAR = 0xfffffffful;
  EPIC->MASK_LEVEL_CLEAR = 0xfffffffful;
	return 0;
}

void scr1_timer_isr();

void __soc_handle_all_irqs(void) {
	unsigned long cause = read_csr(mcause);
	if ( ((cause & CONFIG_RISCV_MCAUSE_EXCEPTION_MASK) == 7) && (cause & (RISCV_MCAUSE_IRQ_BIT)) ) {
		clear_csr(mip, MIP_MTIP);
		scr1_timer_isr();
	} else if ( ((cause & CONFIG_RISCV_MCAUSE_EXCEPTION_MASK) == 11) && (cause & (RISCV_MCAUSE_IRQ_BIT)) ) {
		volatile uint32_t status = EPIC->STATUS;
		clear_csr(mip, MIP_MEIP);
		for (int i = 0; i < 32; i ++) {
			if (status & (1 << i)) {
				_sw_isr_table[i].isr(_sw_isr_table[i].arg);
			}
		}
		EPIC->CLEAR = 0xfffffffful;
  } else {
    k_panic();
  }
}

SYS_INIT(epic_init, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);
