/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_usart

#include <errno.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <hal/mik32/peripherals/Include/mik32_hal_usart.h>
#include <hal/mik32/peripherals/Include/mik32_hal_irq.h>
#include <hal/mik32/shared/include/mik32_memory_map.h>

struct usart_mik32_config {
	UART_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t current_speed;
	uint8_t parity;
	uint32_t clock_id;
	const struct pinctrl_dev_config *pin_cfg;
};

struct usart_mik32_data {
};

static int usart_mik32_init(const struct device *dev)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;
	uint32_t ctrl1 = UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_UE_M;
	uint32_t clock_rate;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t *)(uintptr_t)config->clock_id;
	uint32_t divn;
	int err;

	clock_control_on(config->clock_dev, clock_sys);

	err = clock_control_get_rate(config->clock_dev, clock_sys, &clock_rate);
	if (err != 0) {
		return err;
	}
	divn = (clock_rate + config->current_speed / 2) / config->current_speed;

	switch (config->parity) {
	case UART_CFG_PARITY_NONE:
		break;
	case UART_CFG_PARITY_ODD:
		ctrl1 |= UART_CONTROL1_PCE_M | UART_CONTROL1_PS_M;
		break;
	case UART_CFG_PARITY_EVEN:
		ctrl1 |= UART_CONTROL1_PCE_M;
		break;
	default:
		return -EINVAL;
	}

	regs->DIVIDER = divn;
	regs->CONTROL1 = ctrl1;
	regs->CONTROL2 = 0;
	regs->CONTROL3 = 0;

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

	return 0;
}

static int usart_mik32_poll_in(const struct device *dev, unsigned char *ch)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;

	if ((regs->FLAGS & UART_FLAGS_RXNE_M) == 0) {
		return -1;
	}

	*ch = regs->RXDATA;
	return 0;
}

static void usart_mik32_poll_out(const struct device *dev, unsigned char ch)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;

	while ((regs->FLAGS & UART_FLAGS_TXE_M) == 0) {
	}

	regs->TXDATA = ch;
}

static int usart_mik32_err_check(const struct device *dev)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;
	uint32_t flags = regs->FLAGS;
	enum uart_rx_stop_reason errors = 0;

	if ((flags & UART_FLAGS_PE_M) != 0) {
		errors |= UART_ERROR_PARITY;
	}
	if ((flags & UART_FLAGS_FE_M) != 0) {
		errors |= UART_ERROR_FRAMING;
	}
	if ((flags & UART_FLAGS_NF_M) != 0) {
		errors |= UART_ERROR_NOISE;
	}
	if ((flags & UART_FLAGS_ORE_M) != 0) {
		errors |= UART_ERROR_OVERRUN;
	}

	return errors;
}

static DEVICE_API(uart, usart_mik32_driver_api) = {
	.poll_in = usart_mik32_poll_in,
	.poll_out = usart_mik32_poll_out,
	.err_check = usart_mik32_err_check,
};

#define USART_MIK32_INIT(idx)						\
	PINCTRL_DT_INST_DEFINE(idx);					\
	static struct usart_mik32_data usart_mik32_##idx##_data;	\
	static const struct usart_mik32_config usart_mik32_##idx##_config = { \
		.regs = (UART_TypeDef *)DT_INST_REG_ADDR(idx),		\
		.current_speed = DT_INST_PROP(idx, current_speed),	\
		.parity = DT_INST_ENUM_IDX_OR(idx, parity, UART_CFG_PARITY_NONE), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),	\
		.clock_id = DT_INST_CLOCKS_CELL(idx, id),		\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),		\
	};								\
	DEVICE_DT_INST_DEFINE(idx, &usart_mik32_init, NULL, &usart_mik32_##idx##_data, \
			      &usart_mik32_##idx##_config, PRE_KERNEL_1, \
			      CONFIG_SERIAL_INIT_PRIORITY, &usart_mik32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(USART_MIK32_INIT)
