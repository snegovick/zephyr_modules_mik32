/*
 * Copyright (c) 2018-2023 Intel Corporation
 * Copyright (c) 2025 Escave
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Based on zephyr std riscv_machine_timer.c

#include <limits.h>

#include <zephyr/init.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/sys_clock.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#include <hal/mik32/shared/periphery/scr1_timer.h>
#include <hal/mik32/shared/include/mik32_memory_map.h>
#include <hal/mik32/shared/include/scr1_csr_encoding.h>
#include <soc/mikron/mik32/soc.h>

/* syntacore,machine-timer*/
#if DT_HAS_COMPAT_STATUS_OKAY(syntacore_machine_timer)
#define DT_DRV_COMPAT syntacore_machine_timer
#define MTIMER_HAS_DIVIDER

#define MTIMEDIV_REG	(DT_INST_REG_ADDR_U64(0) + 4)
#define MTIME_REG	(DT_INST_REG_ADDR_U64(0) + 8)
#define MTIMECMP_REG	(DT_INST_REG_ADDR_U64(0) + 16)
//#define TIMER_IRQN	DT_INST_IRQN(0)
#endif

#define CYC_PER_TICK (uint32_t)(sys_clock_hw_cycles_per_sec() \
				/ CONFIG_SYS_CLOCK_TICKS_PER_SEC)

/* the unsigned long cast limits divisions to native CPU register width */
#define cycle_diff_t unsigned long
#define CYCLE_DIFF_MAX (~(cycle_diff_t)0)

#define MIE_MTIE                    (0x1 << 7)

/*
 * We have two constraints on the maximum number of cycles we can wait for.
 *
 * 1) sys_clock_announce() accepts at most INT32_MAX ticks.
 *
 * 2) The number of cycles between two reports must fit in a cycle_diff_t
 *    variable before converting it to ticks.
 *
 * Then:
 *
 * 3) Pick the smallest between (1) and (2).
 *
 * 4) Take into account some room for the unavoidable IRQ servicing latency.
 *    Let's use 3/4 of the max range.
 *
 * Finally let's add the LSB value to the result so to clear out a bunch of
 * consecutive set bits coming from the original max values to produce a
 * nicer literal for assembly generation.
 */
#define CYCLES_MAX_1	((uint64_t)INT32_MAX * (uint64_t)CYC_PER_TICK)
#define CYCLES_MAX_2	((uint64_t)CYCLE_DIFF_MAX)
#define CYCLES_MAX_3	MIN(CYCLES_MAX_1, CYCLES_MAX_2)
#define CYCLES_MAX_4	(CYCLES_MAX_3 / 2 + CYCLES_MAX_3 / 4)
#define CYCLES_MAX	(CYCLES_MAX_4 + LSB_GET(CYCLES_MAX_4))

static struct k_spinlock lock;
static uint64_t last_count;
static uint64_t last_ticks;
static uint32_t last_elapsed;

static uintptr_t get_hart_mtimecmp(void)
{
	return MTIMECMP_REG;
}

static void set_mtimecmp(uint64_t time)
{
	volatile uint32_t *r = (uint32_t *)get_hart_mtimecmp();

	/* Per spec, the RISC-V MTIME/MTIMECMP registers are 64 bit,
	 * but are NOT internally latched for multiword transfers.  So
	 * we have to be careful about sequencing to avoid triggering
	 * spurious interrupts: always set the high word to a max
	 * value first.
	 */
	r[1] = 0xffffffff;
	r[0] = (uint32_t)time;
	r[1] = (uint32_t)(time >> 32);
}

static void set_divider(void)
{
#ifdef MTIMER_HAS_DIVIDER
	*(volatile uint32_t *)MTIMEDIV_REG =
		CONFIG_RISCV_SCR1_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER;
#endif
}

static uint64_t mtime(void)
{
	volatile uint32_t *r = (uint32_t *)MTIME_REG;
	uint32_t lo, hi;

	/* Likewise, must guard against rollover when reading */
	do {
		hi = r[1];
		lo = r[0];
	} while (r[1] != hi);

	return (((uint64_t)hi) << 32) | lo;
}

void scr1_timer_isr()
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	uint64_t now = mtime();
	uint64_t dcycles = now - last_count;
	uint32_t dticks = (cycle_diff_t)dcycles / CYC_PER_TICK;

	last_count += (cycle_diff_t)dticks * CYC_PER_TICK;
	last_ticks += dticks;
	last_elapsed = 0;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		uint64_t next = last_count + CYC_PER_TICK;

		set_mtimecmp(next);
	}

	k_spin_unlock(&lock, key);
	sys_clock_announce(dticks);
}

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
	return ((uint32_t)mtime()) * (CONFIG_RISCV_SCR1_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER + 1);
}

uint64_t sys_clock_cycle_get_64(void)
{
	return mtime() * (CONFIG_RISCV_SCR1_MACHINE_TIMER_SYSTEM_CLOCK_DIVIDER + 1);
}

static int sys_clock_driver_init(void)
{
	set_divider();

	last_ticks = mtime() / CYC_PER_TICK;
	last_count = last_ticks * CYC_PER_TICK;
	set_mtimecmp(last_count + CYC_PER_TICK);

	SCR1_TIMER->TIMER_CTRL |= SCR1_TIMER_CTRL_ENABLE_M;
	set_csr(mie, MIE_MTIE);
	return 0;
}

SYS_INIT(sys_clock_driver_init, PRE_KERNEL_2,
	 CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
