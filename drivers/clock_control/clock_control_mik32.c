/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_mik32_pmgr

#include <stdint.h>

#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <dts/riscv/mikron/mik32/mik32-clocks.h>


/** Power manager offset (from id cell) */
#define MIK32_CLOCK_ID_OFFSET(id) (((id) >> 6U) & 0xFFU)
/** Power manager configuration bit (from id cell) */
#define MIK32_CLOCK_ID_BIT(id)	 ((id)&0x1FU)

#define GET_MUX_TIM(reg, bit_offset) (((reg) >> (bit_offset)) & 0x7)

#define CPU_FREQ DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define HSI32M_FREQ 32000000
#define OSC32K_FREQ 32768
#define LSI32K_FREQ 32768

struct clock_control_mik32_config {
	uint32_t base;
};

/* struct clock_control_mik32_data { */
/* 	uint32_t ahb_set; */
/* 	uint32_t apbm_set; */
/* 	uint32_t apbp_set; */
/* }; */

/* #if DT_HAS_COMPAT_STATUS_OKAY(mikron_mik32_timer32) */
/* /\* timer identifiers *\/ */
/* #define TIMER_ID_OR_NONE(nodelabel)                                            \ */
/* 	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(nodelabel)),          \ */
/* 		    (DT_CLOCKS_CELL(DT_NODELABEL(nodelabel), id),), ()) */

/* static const uint16_t timer_ids[] = { */
/* 	TIMER_ID_OR_NONE(timer0)  /\* *\/ */
/* 	TIMER_ID_OR_NONE(timer1)  /\* *\/ */
/* 	TIMER_ID_OR_NONE(timer2)  /\* *\/ */
/* }; */
/* #endif /\* DT_HAS_COMPAT_STATUS_OKAY(mikron_mik32_timer32) *\/ */

static int clock_control_mik32_on(const struct device *dev,
				 clock_control_subsys_t sys)
{
	const struct clock_control_mik32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_set_bit(config->base + MIK32_CLOCK_ID_OFFSET(id),
		    MIK32_CLOCK_ID_BIT(id));

	return 0;
}

static int clock_control_mik32_off(const struct device *dev,
				  clock_control_subsys_t sys)
{
	const struct clock_control_mik32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;

	sys_set_bit(config->base + MIK32_CLOCK_ID_OFFSET(id) + 0x4,
		    MIK32_CLOCK_ID_BIT(id));

	return 0;
}

static int clock_control_mik32_get_rate(const struct device *dev,
				       clock_control_subsys_t sys,
				       uint32_t *rate)
{
	const struct clock_control_mik32_config *config = dev->config;
	uint16_t id = *(uint16_t *)sys;
	uint8_t psc;
	uint32_t ahb_psc = sys_read32(config->base + MIK32_PMGR_DIV_AHB_OFFSET);
	uint32_t ahb_rate = CPU_FREQ / (ahb_psc + 1);

	switch (MIK32_CLOCK_ID_OFFSET(id)) {
	case MIK32_PMGR_AHB_SET_OFFSET:
		*rate = ahb_rate;
		break;
	case MIK32_PMGR_APBM_SET_OFFSET:
		psc = sys_read32(config->base + MIK32_PMGR_DIV_APBM_OFFSET);
		*rate = CPU_FREQ / (psc + 1);
		break;
	case MIK32_PMGR_APBP_SET_OFFSET:
		psc = sys_read32(config->base + MIK32_PMGR_DIV_APBP_OFFSET);
		*rate = CPU_FREQ / (psc + 1);
		break;
	default:
		return -ENOTSUP;
	}

#if DT_HAS_COMPAT_STATUS_OKAY(mikron_mik32_timer)
	/* handle timer clocks */
	unsigned int bit_offset = 0;
	uint8_t mux = 0;
	bool is_t32 = false;
	if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBM_SET_OFFSET) &&
	    (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER32_0)) {
		bit_offset = 0;
		mux = GET_MUX_TIM(config->reg + MIK32_PMGR_TIMER_CFG_OFFSET, bit_offset);
		
	} else if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBP_SET_OFFSET) &&
		   (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER16_0)) {
		bit_offset = 9;
		is_t32 = false;
	} else if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBP_SET_OFFSET) &&
		   (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER16_1)) {
		bit_offset = 12;
		is_t32 = false;
	} else if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBP_SET_OFFSET) &&
		   (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER16_2)) {
		bit_offset = 15;
		is_t32 = false;
	} else if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBP_SET_OFFSET) &&
		   (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER32_1)) {
		bit_offset = 3;
		is_t32 = true;
	} else if ((MIK32_CLOCK_ID_OFFSET(id) == MIK32_PMGR_APBP_SET_OFFSET) &&
		   (MIK32_CLOCK_ID_BIT(id) == MIK32_CLOCK_TIMER32_2)) {
		bit_offset = 6;
		is_t32 = true;
	}

	if (is_t32) {
		switch (mux) {
		case 0:
			*rate = CPU_FREQ;
			break;
		case 1:
			*rate = ahb_rate;
			break;
		case 2:
			*rate = LSI32K_FREQ;
			break;
		}
	} else {
		switch (mux) {
		case 0:
			*rate = CPU_FREQ;
			break;
		case 1:
			*rate = ahb_rate;
			break;
		case 2:
			*rate = CPU_FREQ;
			break;
		case 3:
			*rate = HSI32M_FREQ;
			break;
		case 4:
			*rate = OSC32K_FREQ;
			break;
		case 5:
			*rate = LSI32K_FREQ;
			break;
		}		
	}

#endif /* DT_HAS_COMPAT_STATUS_OKAY(mikron_mik32_timer) */

	return 0;
}

static enum clock_control_status
clock_control_mik32_get_status(const struct device *dev,
			      clock_control_subsys_t sys)
{
	const struct clock_control_mik32_config *config = dev->config;
	//struct clock_control_mik32_data *data = dev->data;
	uint16_t id = *(uint16_t *)sys;

	if (sys_test_bit(config->base + MIK32_CLOCK_ID_OFFSET(id),
			 MIK32_CLOCK_ID_BIT(id)) != 0) {
		return CLOCK_CONTROL_STATUS_ON;
	}

	return CLOCK_CONTROL_STATUS_OFF;
}

static DEVICE_API(clock_control, clock_control_mik32_api) = {
	.on = clock_control_mik32_on,
	.off = clock_control_mik32_off,
	.get_rate = clock_control_mik32_get_rate,
	.get_status = clock_control_mik32_get_status,
};

/* static const struct clock_control_mik32_data data = { */
/* 	.ahb_set = 0x1f, */
/* 	.apbm_set = 0x89, */
/* 	.apbp_set = 0x00, */
/* }; */

#define CLOCK_CONTROL_MIK32_INIT(idx)					\
	static const struct clock_control_mik32_config clock_control_mik32_##idx##_config = { \
		.base = DT_INST_REG_ADDR(idx),				\
	};								\
									\
	DEVICE_DT_INST_DEFINE(idx, NULL, NULL, NULL, &clock_control_mik32_##idx##_config, PRE_KERNEL_1, \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,	\
			      &clock_control_mik32_api);

DT_INST_FOREACH_STATUS_OKAY(CLOCK_CONTROL_MIK32_INIT)
