/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <limits.h>

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#include <hal/mik32/peripherals/Include/mik32_hal_timer32.h>

#define CYC_PER_TICK (uint32_t)(sys_clock_hw_cycles_per_sec() \
				/ CONFIG_SYS_CLOCK_TICKS_PER_SEC)

static struct k_spinlock lock;
static uint64_t last_count;
static uint64_t last_ticks;
static uint32_t last_elapsed;

void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t cyc;

	if (ticks == K_TICKS_FOREVER) {
		cyc = last_count + CYCLES_MAX;
	} else {
		cyc = (last_ticks + last_elapsed + ticks) * CYC_PER_TICK;
		if ((cyc - last_count) > CYCLES_MAX) {
			cyc = last_count + CYCLES_MAX;
		}
	}
	set_mtimecmp(cyc);

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t now = mtime();
	uint64_t dcycles = now - last_count;
	uint32_t dticks = (cycle_diff_t)dcycles / CYC_PER_TICK;

	last_elapsed = dticks;
	k_spin_unlock(&lock, key);
	return dticks;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return ((uint32_t)mtime()) << CONFIG_RISCV_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER;
}

uint64_t sys_clock_cycle_get_64(void)
{
	return mtime() << CONFIG_RISCV_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER;
}

static int sys_clock_driver_init(void)
{

	IRQ_CONNECT(TIMER_IRQN, 0, timer_isr, NULL, 0);
	last_ticks = mtime() / CYC_PER_TICK;
	last_count = last_ticks * CYC_PER_TICK;
	set_mtimecmp(last_count + CYC_PER_TICK);
	irq_enable(TIMER_IRQN);
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
