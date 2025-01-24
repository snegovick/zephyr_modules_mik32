/*
 * Copyright (c) 2025 Excave.ru
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_usart

#include <errno.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/soc/mik32_memory_map.h>
#include <zephyr/soc/mik32_irq.h>
#include <zephyr/irq.h>

#include "mik32_uart.h"

struct usart_mik32_config {
	UART_TypeDef *regs;
	const struct device *clock_dev;
	uint32_t current_speed;
	uint8_t parity;
	uint32_t clock_id;
	const struct pinctrl_dev_config *pin_cfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct usart_mik32_data {
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_mik32_isr(const struct device *dev)
{
	struct usart_mik32_data *const data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int usart_mik32_init(const struct device *dev)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;
	uint32_t ctrl1 = UART_CONTROL1_TE_M | UART_CONTROL1_RE_M | UART_CONTROL1_UE_M;
	uint32_t clock_rate;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t)&config->clock_id;
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
	regs->CONTROL2 = 0;
	regs->CONTROL3 = 0;
	regs->CONTROL1 = ctrl1;

	err = pinctrl_apply_state(config->pin_cfg, PINCTRL_STATE_DEFAULT);
	if (err != 0) {
		return err;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int usart_mik32_poll_in(const struct device *dev, unsigned char *ch)
{
	const struct usart_mik32_config *config = dev->config;
	UART_TypeDef *regs = config->regs;

	uint32_t flags = regs->FLAGS;

	if ((flags & UART_FLAGS_RXNE_M) == 0) {
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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
int usart_mik32_fifo_fill(const struct device *dev, const uint8_t *tx_data,
			 int len)
{
	usart_mik32_poll_out(dev, tx_data[0]);
	return 1;
}

int usart_mik32_fifo_read(const struct device *dev, uint8_t *rx_data,
			 const int size)
{
	usart_mik32_poll_in(dev, rx_data);
	return 1;
}

void usart_mik32_irq_tx_enable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL1 |= UART_CONTROL1_TCIE_M;
}

void usart_mik32_irq_tx_disable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL1 &= ~UART_CONTROL1_TCIE_M;
}

int usart_mik32_irq_tx_ready(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	if ((regs->FLAGS & UART_FLAGS_TXE_M) != 0) {
		return 1;
	}
	return 0;
}

int usart_mik32_irq_tx_complete(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	if ((regs->FLAGS & UART_FLAGS_TC_M) != 0) {
		return 1;
	}
	return 0;
}

void usart_mik32_irq_rx_enable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL1 |= UART_CONTROL1_RXNEIE_M;
}

void usart_mik32_irq_rx_disable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL1 &= ~UART_CONTROL1_RXNEIE_M;
}

int usart_mik32_irq_rx_ready(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	if ((regs->FLAGS & UART_FLAGS_RXNE_M) != 0) {
		return 1;
	}
	return 0;
}

void usart_mik32_irq_err_enable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL3 |= UART_CONTROL3_EIE_M;
}

void usart_mik32_irq_err_disable(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	regs->CONTROL3 &= ~UART_CONTROL3_EIE_M;
}

int usart_mik32_irq_is_pending(const struct device *dev)
{
	const struct usart_mik32_config *const config = dev->config;
	UART_TypeDef *regs = config->regs;

	if (((regs->FLAGS & UART_FLAGS_TC_M) != 0) ||
	    (((regs->FLAGS & UART_FLAGS_RXNE_M) != 0))) {
		return 1;
	}
	return 0;
}

int usart_mik32_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

void usart_mik32_irq_callback_set(const struct device *dev,
				 uart_irq_callback_user_data_t cb,
				 void *user_data)
{
	struct usart_mik32_data *const data = dev->data;

	data->user_cb = cb;
	data->user_data = user_data;
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


static DEVICE_API(uart, usart_mik32_driver_api) = {
	.poll_in = usart_mik32_poll_in,
	.poll_out = usart_mik32_poll_out,
	.err_check = usart_mik32_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_mik32_fifo_fill,
	.fifo_read = usart_mik32_fifo_read,
	.irq_tx_enable = usart_mik32_irq_tx_enable,
	.irq_tx_disable = usart_mik32_irq_tx_disable,
	.irq_tx_ready = usart_mik32_irq_tx_ready,
	.irq_tx_complete = usart_mik32_irq_tx_complete,
	.irq_rx_enable = usart_mik32_irq_rx_enable,
	.irq_rx_disable = usart_mik32_irq_rx_disable,
	.irq_rx_ready = usart_mik32_irq_rx_ready,
	.irq_err_enable = usart_mik32_irq_err_enable,
	.irq_err_disable = usart_mik32_irq_err_disable,
	.irq_is_pending = usart_mik32_irq_is_pending,
	.irq_update = usart_mik32_irq_update,
	.irq_callback_set = usart_mik32_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define MIK32_USART_IRQ_HANDLER(n)					\
	static void usart_mik32_config_func_##n(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(n),				\
			    DT_INST_IRQ(n, 0),				\
			    usart_mik32_isr,				\
			    DEVICE_DT_INST_GET(n),			\
			    0);						\
		irq_enable(DT_INST_IRQN(n));				\
	}
#define MIK32_USART_IRQ_HANDLER_FUNC_INIT(n)		\
	.irq_config_func = usart_mik32_config_func_##n
#else /* CONFIG_UART_INTERRUPT_DRIVEN */
#define MIK32_USART_IRQ_HANDLER(n)
#define MIK32_USART_IRQ_HANDLER_FUNC_INIT(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define USART_MIK32_INIT(idx)						\
	PINCTRL_DT_INST_DEFINE(idx);					\
	MIK32_USART_IRQ_HANDLER(idx)					\
	static struct usart_mik32_data usart_mik32_##idx##_data;	\
	static const struct usart_mik32_config usart_mik32_##idx##_config = { \
		.regs = (UART_TypeDef *)DT_INST_REG_ADDR(idx),		\
		.current_speed = DT_INST_PROP(idx, current_speed),	\
		.parity = DT_INST_ENUM_IDX_OR(idx, parity, UART_CFG_PARITY_NONE), \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),	\
		.clock_id = DT_INST_CLOCKS_CELL(idx, id),		\
		.pin_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),		\
		MIK32_USART_IRQ_HANDLER_FUNC_INIT(idx)			\
	};								\
	DEVICE_DT_INST_DEFINE(idx, &usart_mik32_init, NULL, &usart_mik32_##idx##_data, \
			      &usart_mik32_##idx##_config, PRE_KERNEL_1, \
			      CONFIG_SERIAL_INIT_PRIORITY, &usart_mik32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(USART_MIK32_INIT)
