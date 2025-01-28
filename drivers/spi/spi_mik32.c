/*
 * Copyright (c) 2025 Excave.ru
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_mik32_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mik32.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>

#include "spi_mik32.h"

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(spi_mik32);

#include "spi_context.h"

#define MIK32_SPI_PSC_MAX 0x7U

struct spi_mik32_config {
	SPI_TypeDef * regs;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_SPI_MIK32_INTERRUPT
	void (*irq_configure)();
#endif
};

struct spi_mik32_data {
	struct spi_context ctx;
};

static int spi_mik32_get_err(const struct spi_mik32_config *cfg, uint32_t status)
{
	if (status & SPI_INT_STATUS_ERR_M) {
		LOG_ERR("spi@%p error status detected, err = %u",
			cfg->regs, status & (uint32_t)SPI_INT_STATUS_ERR_M);

		return -EIO;
	}

	return 0;
}

static bool spi_mik32_transfer_ongoing(struct spi_mik32_data *data)
{
	return spi_context_tx_on(&data->ctx) ||
	       spi_context_rx_on(&data->ctx);
}

static inline void _spi_clear_txfifo(SPI_TypeDef *regs)
{
	regs->ENABLE |= SPI_ENABLE_CLEAR_TX_FIFO_M;
}

static inline void _spi_clear_rxfifo(SPI_TypeDef *regs)
{
	regs->ENABLE |= SPI_ENABLE_CLEAR_RX_FIFO_M;
}

static inline void _spi_disable(SPI_TypeDef *regs)
{
	regs->ENABLE &= ~SPI_ENABLE_M;
}

static inline void _spi_clear_ints(SPI_TypeDef *regs)
{
	volatile uint32_t status = regs->INT_STATUS;
	(void)status;
}

static inline void _spi_disable_ints(SPI_TypeDef *regs)
{
	regs->INT_DISABLE |= SPI_INT_STATUS_RX_OVERFLOW_M |
		SPI_INT_STATUS_MODE_FAIL_M |
		SPI_INT_STATUS_TX_FIFO_NOT_FULL_M |
		SPI_INT_STATUS_TX_FIFO_FULL_M |
		SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M |
		SPI_INT_STATUS_RX_FIFO_FULL_M |
		SPI_INT_STATUS_TX_FIFO_UNDERFLOW_M;
}

static inline void _spi_enable_ints(SPI_TypeDef *regs)
{
	regs->INT_ENABLE |= SPI_INT_STATUS_RX_OVERFLOW_M | SPI_INT_STATUS_MODE_FAIL_M | SPI_INT_STATUS_TX_FIFO_NOT_FULL_M  | SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M;
}

static inline void _spi_disable_clear_errors(SPI_TypeDef *regs)
{
	_spi_disable(regs);
	_spi_clear_rxfifo(regs);
	_spi_clear_txfifo(regs);
	_spi_clear_ints(regs);
	_spi_disable_ints(regs);
}

static inline void _spi_enable(SPI_TypeDef *regs)
{
	regs->ENABLE |= SPI_ENABLE_M;
}

static inline void _spi_set_cs(SPI_TypeDef *regs, uint32_t cs)
{
	switch (cs) {
	case 0:
		regs->CONFIG |= SPI_CONFIG_CS_0_M;
		break;
	case 1:
		regs->CONFIG |= SPI_CONFIG_CS_1_M;
		break;
	case 2:
		regs->CONFIG |= SPI_CONFIG_CS_2_M;
		break;
	case 3:
		regs->CONFIG |= SPI_CONFIG_CS_3_M;
		break;
	}
}

static int spi_mik32_configure(const struct device *dev,
			      const struct spi_config *config)
{
	struct spi_mik32_data *data = dev->data;
	const struct spi_mik32_config *cfg = dev->config;
	SPI_TypeDef *regs = cfg->regs;
	uint32_t spi_config = 0;

	if (spi_context_configured(&data->ctx, config)) {
		return 0;
	}

	if (SPI_OP_MODE_GET(config->operation) == SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported yet");
		return -ENOTSUP;
	}

	_spi_clear_txfifo(regs);
	_spi_clear_rxfifo(regs);
	_spi_disable(regs);

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Non-8-bit frame size is not supported");
		return -ENOTSUP;
	}

	spi_config = SPI_CONFIG_MASTER_M | SPI_DECODER_NONE;

	if (spi_cs_is_gpio(config)) {
		spi_config |= SPI_CONFIG_MANUAL_CS_M;
		spi_config |= SPI_CS_NONE << SPI_CONFIG_CS_S;
	}

	if (config->operation & SPI_MODE_CPOL) {
		spi_config |= SPI_CONFIG_CLK_POL_M;		
	}

	if (config->operation & SPI_MODE_CPHA) {
		spi_config |= SPI_CONFIG_CLK_PH_M;
	}

	(void)clock_control_get_rate(MIK32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&cfg->clkid,
				     &bus_freq);

	bus_freq = bus_freq >> 1U;
	for (uint8_t i = 1; (1 << (i + 1)) <= SPI_MAXIMUM_BAUD_RATE_DIV; i++) {
		bus_freq = bus_freq >> 1U;
		if (bus_freq <= config->frequency) {
			spi_config |= (((i) & 0x7) << SPI_CONFIG_BAUD_RATE_DIV_S);
			break;
		}
	}

	regs->CONFIG = spi_config;

	data->ctx.config = config;

	return 0;
}

static int spi_mik32_frame_exchange(const struct device *dev)
{
	struct spi_mik32_data *data = dev->data;
	const struct spi_mik32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	SPI_TypeDef *regs = cfg->regs;
	uint16_t tx_frame = 0U, rx_frame = 0U;
	volatile uint32_t status = regs->INT_STATUS;

	while ((status & SPI_INT_STATUS_TX_FIFO_NOT_FULL_M) == 0) {
		status = regs->INT_STATUS;
	}

	if (spi_context_tx_buf_on(ctx)) {
		tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
	}
	regs->TXDATA = tx_frame;
	
	spi_context_update_tx(ctx, 1, 1);

	status = regs->INT_STATUS;
	while ((status & SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M) == 0) {
		status = regs->INT_STATUS;
	}

	rx_frame = regs->RXDATA;
	if (spi_context_rx_buf_on(ctx)) {
		UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
	}

	spi_context_update_rx(ctx, 1, 1);

	status = regs->INT_STATUS;
	return spi_mik32_get_err(cfg, status);
}

static int spi_mik32_transceive_impl(const struct device *dev,
				     const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs,
				     spi_callback_t cb,
				     void *userdata)
{
	struct spi_mik32_data *data = dev->data;
	const struct spi_mik32_config *cfg = dev->config;
	SPI_TypeDef *regs = cfg->regs;
	int ret;

	spi_context_lock(&data->ctx, (cb != NULL), cb, userdata, config);

	ret = spi_mik32_configure(dev, config);
	if (ret < 0) {
		goto error;
	}

	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

	if (spi_cs_is_gpio(config)) {
		_spi_enable(regs);
		spi_context_cs_control(&data->ctx, true);
	} else {
		_spi_set_cs(data->config->slave);
	}

#ifdef CONFIG_SPI_MIK32_INTERRUPT
	_spi_enable_ints(regs);
	ret = spi_context_wait_for_completion(&data->ctx);
#else
	regs->TX_THR = 1;
	do {
		ret = spi_mik32_frame_exchange(dev);
		if (ret < 0) {
			break;
		}
	} while (spi_mik32_transfer_ongoing(data));

#ifdef CONFIG_SPI_ASYNC
	spi_context_complete(&data->ctx, dev, ret);
#endif
#endif

	uint32_t status = regs->INT_STATUS;
	while (status & SPI_INT_STATUS_SPI_ACTIVE_M) {
		/* Wait until last frame transfer complete. */
		status = regs->INT_STATUS;
	}

	spi_context_cs_control(&data->ctx, false);

	_spi_disable_clear_errors(regs);
error:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_mik32_transceive(const struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return spi_mik32_transceive_impl(dev, config, tx_bufs, rx_bufs, NULL, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_mik32_transceive_async(const struct device *dev,
				      const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      spi_callback_t cb,
				      void *userdata)
{
	return spi_mik32_transceive_impl(dev, config, tx_bufs, rx_bufs, cb, userdata);
}
#endif

#ifdef CONFIG_SPI_MIK32_INTERRUPT

static void spi_mik32_complete(const struct device *dev, int status)
{
	struct spi_mik32_data *data = dev->data;
	const struct spi_mik32_config *cfg = dev->config;
	SPI_TypeDef *regs = cfg->regs;

	_spi_disable_clear_errors(regs);

	spi_context_complete(&data->ctx, dev, status);
}

static void spi_mik32_isr(struct device *dev)
{
	const struct spi_mik32_config *cfg = dev->config;
	struct spi_mik32_data *data = dev->data;
	SPI_TypeDef *regs = cfg->regs;
	int err = 0;

	volatile uint32_t status = regs->INT_STATUS;

	err = spi_mik32_get_err(cfg, status);
	if (err) {
		spi_mik32_complete(dev, err);
		return;
	}

	if (spi_mik32_transfer_ongoing(data)) {
		if (status & SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M) {
			uint32_t rx_frame = regs->RXDATA;
			if (spi_context_rx_buf_on(ctx)) {
				UNALIGNED_PUT(rx_frame, (uint8_t *)data->ctx.rx_buf);
			}
			spi_context_update_rx(ctx, 1, 1);
		}

		if (status & SPI_INT_STATUS_TX_FIFO_NOT_FULL_M) {
			uint32_t tx_frame = 0;
			if (spi_context_tx_buf_on(ctx)) {
				tx_frame = UNALIGNED_GET((uint8_t *)(data->ctx.tx_buf));
			}
			regs->TXDATA = tx_frame;
			spi_context_update_tx(ctx, 1, 1);
		}
	}

	if (err || !spi_mik32_transfer_ongoing(data)) {
		spi_mik32_complete(dev, err);
	}
}

#endif /* SPI_MIK32_INTERRUPT */

static int spi_mik32_release(const struct device *dev,
			    const struct spi_config *config)
{
	struct spi_mik32_data *data = dev->data;

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_mik32_driver_api) = {
	.transceive = spi_mik32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_mik32_transceive_async,
#endif
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
	.release = spi_mik32_release
};

int spi_mik32_init(const struct device *dev)
{
	struct spi_mik32_data *data = dev->data;
	const struct spi_mik32_config *cfg = dev->config;
	int ret;

	(void)clock_control_on(MIK32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Failed to apply pinctrl state");
		return ret;
	}

	ret = spi_context_cs_configure_all(&data->ctx);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_SPI_MIK32_INTERRUPT
	cfg->irq_configure(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#define MIK32_IRQ_CONFIGURE(idx)					   \
	static void spi_mik32_irq_configure_##idx(void)			   \
	{								   \
		IRQ_CONNECT(DT_INST_IRQN(idx), DT_INST_IRQ(idx, priority), \
			    spi_mik32_isr,				   \
			    DEVICE_DT_INST_GET(idx), 0);		   \
		irq_enable(DT_INST_IRQN(idx));				   \
	}

#define MIK32_SPI_INIT(idx)						       \
	PINCTRL_DT_INST_DEFINE(idx);					       \
	IF_ENABLED(CONFIG_SPI_MIK32_INTERRUPT, (MIK32_IRQ_CONFIGURE(idx)));    \
	static struct spi_mik32_data spi_mik32_data_##idx = {		       \
		SPI_CONTEXT_INIT_LOCK(spi_mik32_data_##idx, ctx),	       \
		SPI_CONTEXT_INIT_SYNC(spi_mik32_data_##idx, ctx),	       \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx) };      \
	static struct spi_mik32_config spi_mik32_config_##idx = {              \
		.divisor = DT_INST_PROP(idx, divider),		               \
		.regs = (SPI_TypeDef *)DT_INST_REG_ADDR(idx),                  \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),			       \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),		       \
		IF_ENABLED(CONFIG_SPI_MIK32_INTERRUPT,			       \
			   (.irq_configure = spi_mik32_irq_configure_##idx)) }; \
	SPI_DEVICE_DT_INST_DEFINE(idx, spi_mik32_init, NULL,		       \
			      &spi_mik32_data_##idx, &spi_mik32_config_##idx,  \
			      POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,	       \
			      &spi_mik32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MIK32_SPI_INIT)
