/*
 * Copyright (c) 2025 Excave.ru
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT mikron_mik32_i2c

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mik32.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/i2c.h>

#include <mik32_i2c.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_mik32, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mik32.h"

struct i2c_mik32_config {
	I2C_TypeDef * regs;
	uint32_t bitrate;
	uint16_t clkid;
	struct reset_dt_spec reset;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_func)(void);
};

struct i2c_mik32_data {
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	uint32_t dev_config;
	uint16_t addr1;
	uint16_t addr2;
	uint16_t addr1_mask;
	uint16_t addr2_mask;
	uint32_t xfer_len;
	uint32_t xfered_len;
	struct i2c_msg *current;
	uint8_t errs;
	bool is_restart;
	bool master_active;
};

static inline void i2c_mik32_enable_tx_interrupts(const struct i2c_mik32_config  *cfg)
{
	I2C_TypeDef *regs = cfg->reg;
	regs->CR1 |= I2C_CR1_ERRIE_M | I2C_CR1_TCIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TXIE_M;
}

static inline void i2c_mik32_enable_rx_interrupts(const struct i2c_mik32_config  *cfg)
{
	I2C_TypeDef *regs = cfg->reg;
	regs->CR1 |= I2C_CR1_ERRIE_M | I2C_CR1_TCIE_M | I2C_CR1_ADDRIE_M | I2C_CR1_RXIE_M;
}

static inline void i2c_mik32_disable_interrupts(const struct i2c_mik32_config  *cfg)
{
	I2C_TypeDef *regs = cfg->reg;
	regs->CR1 &= ~I2C_INTMASK;
}

static void i2c_mik32_isr(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	uint32_t stat;

	I2C_TypeDef *regs = cfg->reg;

	uint32_t int_mask = regs->CR1 & I2C_INTMASK; /* разрешенные прерывания  */
	uint32_t status = regs->ISR; /* Флаги */

	if ((status & I2C_ISR_ADDR_M) && (int_mask & I2C_CR1_ADDRIE_M)) {
		if (regs->CR1 & I2C_CR1_SBC_M) {
			regs->CR2 &= ~I2C_CR2_NBYTES_M;
			regs->CR2 |= I2C_CR2_NBYTES(0x1);
		}

		/* Сброс флага ADDR */
		regs->ICR |= I2C_ICR_ADDRCF_M;
	}

	if ((status & (I2C_ISR_BERR_M | I2C_ISR_ARLO_M | I2C_ISR_OVR_M)) && (int_mask & I2C_CR1_ERRIE_M)) {
		/* Выключить все прерывания I2C */
		i2c_mik32_disable_interrupts(cfg);
		/* Сброс I2C */
		if (status & I2C_ISR_BERR_M) {
			data->errs |= I2C_ERROR_BERR;
		}
		if (status & I2C_ISR_ARLO_M) {
			data->errs |= I2C_ERROR_ARLO;			
		}
		if (status & I2C_ISR_OVR_M) {
			data->errs |= I2C_ERROR_OVR;			
		}
		regs->CR1 &= ~I2C_CR1_PE_M;
		regs->CR1 |= I2C_CR1_PE_M;
	}

	if ((status & I2C_ISR_NACKF_M) && (int_mask & I2C_CR1_NACKIE_M)) {
		/* Выключить все прерывания I2C */
		i2c_mik32_disable_interrupts(cfg);
		/* Сброс I2C */
		data->errs = I2C_ERROR_NACK;
		regs->CR1 &= ~I2C_CR1_PE_M;
		regs->CR1 |= I2C_CR1_PE_M;
	}

	if ((status & I2C_ISR_STOPF_M) && (int_mask & I2C_CR1_STOPIE_M)) {
		/* Сброс содержимого TXDR */
		regs->ISR |= I2C_ISR_TXE_M;
		/* Сброс флага детектирования STOP на шине */
		regs->ICR |= I2C_ICR_STOPCF_M;

		regs->CR2 |= I2C_CR2_STOP_M;
		k_sem_give(&data->sync_sem);
	}

	if ((status & I2C_ISR_TXIS_M) && (int_mask & I2C_CR1_TXIE_M)) {
		data->xfered_len++;
		if ((data->xfered_len > data->xfer_len) && (data->master_active == false))
		{
			regs->CR1 &= ~I2C_CR1_PE_M;
			regs->CR1 |= I2C_CR1_PE_M;

			regs->CR2 |= I2C_CR2_STOP_M;
			k_sem_give(&data->sync_sem);
		}
		else
		{
			regs->TXDR = *((uint8_t *)data->current->buf);
			data->current->buf++;
			if (data->xfered_len == data->xfer_len)
			{
				regs->CR2 |= I2C_CR2_STOP_M;
				k_sem_give(&data->sync_sem);
			}
		}
	}

	if ((status & I2C_ISR_RXNE_M) && (int_mask & I2C_CR1_RXIE_M)) {
		*((uint8_t *)data->current->buf) = (uint8_t)regs->RXDR;

		if (regs->CR1 & I2C_CR1_SBC_M)
		{
			regs->CR2 &= ~I2C_CR2_NACK_M; /* Формирование ACK */
		}

		data->current->buf++;
		data->xfered_len++;
		if (data->xfered_len == data->xfer_len)
		{
			regs->CR2 |= I2C_CR2_STOP_M;
			k_sem_give(&data->sync_sem);
		}
	}

	if ((status & I2C_ISR_TCR_M) && (int_mask & I2C_CR1_TCIE_M)) {
		if (regs->CR1 & I2C_CR1_SBC_M)
		{
			*((uint8_t *)data->current->buf) = (uint8_t)regs->RXDR;
			data->current->buf++;
			data->xfered_len++;

			regs->CR2 &= ~I2C_CR2_NACK_M; /* Формирование ACK */
			/* Выключить все прерывания I2C */
			i2c_mik32_disable_interrupts(cfg);
			/* Сброс I2C */
			regs->CR1 &= ~I2C_CR1_PE_M;
			regs->CR1 |= I2C_CR1_PE_M;

			if (data->xfered_len < data->xfer_len)
			{
				regs->CR2 &= ~I2C_CR2_NBYTES_M;
				regs->CR2 |= I2C_CR2_NBYTES(0x1);
			}
			else
			{
				regs->CR1 &= ~(I2C_CR1_TXIE_M);
				regs->CR2 |= I2C_CR2_STOP_M;
				k_sem_give(&data->sync_sem);
			}
		}
		else
		{
			regs->CR2 &= ~I2C_CR2_NBYTES_M;
			/* Подготовка перед отправкой */
		
			if ((data->xfer_len - data->xfered_len) <= I2C_NBYTE_MAX)
			{
				regs->CR2 &= ~I2C_CR2_NBYTES_M;
				regs->CR2 |= I2C_CR2_NBYTES(data->xfer_len - data->xfered_len);
				regs->CR2 &= ~I2C_CR2_RELOAD_M;
				regs->CR2 &= ~I2C_CR2_AUTOEND_M;
				//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;
			}
			else /* DataSize > 255 */
			{
				regs->CR2 &= ~I2C_CR2_NBYTES_M;
				regs->CR2 |= I2C_CR2_NBYTES(I2C_NBYTE_MAX);
				/* При RELOAD = 1 AUTOEND игнорируется */
				regs->CR2 |= I2C_CR2_RELOAD_M;
			}
		}
	}

	if ((status & I2C_ISR_TC_M) && (int_mask & I2C_CR1_TCIE_M)) {
		hi2c->State = HAL_I2C_STATE_END;
	}

	if (data->errs != 0U) {
		/* Enter stop condition */
		regs->CR2 |= I2C_CR2_STOP_M;

		k_sem_give(&data->sync_sem);
	}
}

static void i2c_mik32_log_err(struct i2c_mik32_data *data)
{
	if (data->errs & I2C_ERROR_BERR) {
		LOG_ERR("Bus error");
	}

	if (data->errs & I2C_ERROR_ARLO) {
		LOG_ERR("Arbitration lost");
	}

	if (data->errs & I2C_ERROR_NACK) {
		LOG_ERR("NACK received");
	}

	if (data->errs & I2C_ERROR_STOP) {
		LOG_ERR("I2C bus busy");
	}

	if (data->errs & I2C_ERROR_OVR) {
		LOG_ERR("Overrun error");
	}
}

static void i2c_mik32_xfer_begin(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	I2C_TypeDef *regs = cfg->regs;

	k_sem_reset(&data->sync_sem);

	data->errs = 0U;
	data->is_restart = false;
	data->xfered_len = 0;

	i2c_mik32_disable_interrupts(cfg);
	if ((data->xfer_len - data->xfered_len) <= I2C_NBYTE_MAX) {
		regs->CR2 &= ~I2C_CR2_NBYTES_M;
		regs->CR2 |= I2C_CR2_NBYTES(data->xfer_len - data->xfered_len);
		regs->CR2 &= ~I2C_CR2_RELOAD_M;
		regs->CR2 &= ~I2C_CR2_AUTOEND_M;
		//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;
	} else {
		regs->CR2 &= ~I2C_CR2_NBYTES_M;
		regs->CR2 |= I2C_CR2_NBYTES(I2C_NBYTE_MAX);
		/* При RELOAD = 1 AUTOEND игнорируется */
		regs->CR2 |= I2C_CR2_RELOAD_M;
	}

	if (data->current->flags & I2C_MSG_READ) {
		regs->CR2 |= I2C_CR2_RD_WRN_M;
		i2c_mik32_enable_rx_interrupts(cfg);
	} else {
		regs->CR2 &= ~I2C_CR2_RD_WRN_M;
		i2c_mik32_enable_tx_interrupts(cfg);
	}

	/* Enter start condition */
	regs->CR2 |= I2C_CR2_START_M;
}

static int i2c_mik32_xfer_end(const struct device *dev)
{
	struct i2c_gd32_data *data = dev->data;
	const struct i2c_gd32_config *cfg = dev->config;
	I2C_TypeDef *regs = cfg->regs;

	i2c_mik32_disable_interrupts(cfg);

	/* Wait for stop condition is done. */
	while (regs->ISR & I2C_ISR_BUSY_M) {
		/* NOP */
	}

	if (data->errs) {
		return -EIO;
	}

	return 0;
}

static int i2c_mik32_msg_write(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	I2C_TypeDef *regs = cfg->regs;

	if (regs->ISR & I2C_ISR_BUSY_M) {
		data->errs = I2C_MIK32_ERR_BUSY;
		return -EBUSY;
	}

	i2c_mik32_xfer_begin(dev);

	k_sem_take(&data->sync_sem, K_FOREVER);

	return i2c_mik32_xfer_end(dev);
}

static int i2c_mik32_transfer(const struct device *dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	I2C_TypeDef *regs = cfg->regs;
	struct i2c_msg *current, *next;
	uint8_t itr;
	int err = 0;

	current = msgs;

	/* First message flags implicitly contain I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {

		if (i < num_msgs) {
			next = current + 1;

			/*
			 * If there have a R/W transfer state change between messages,
			 * An explicit I2C_MSG_RESTART flag is needed for the second message.
			 */
			if ((current->flags & I2C_MSG_RW_MASK) !=
			(next->flags & I2C_MSG_RW_MASK)) {
				if ((next->flags & I2C_MSG_RESTART) == 0U) {
					return -EINVAL;
				}
			}

			/* Only the last message need I2C_MSG_STOP flag to free the Bus. */
			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}
		}

		if ((current->buf == NULL) ||
		    (current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Enable i2c device */
	regs->CR1 |= I2C_CR1_PE_M;

	if (data->dev_config & I2C_ADDR_10_BITS) {
		regs->CR2 |= I2C_CR2_ADD10_M;
		regs->CR2 &= ~I2C_CR2_HEAD10R_M; /* ведущий отправляет полную последовательность для чтения для 10 битного адреса */
		regs->CR2 |= (addr & 0x3FF) << I2C_CR2_SADD_S;
	} else {
		regs->CR2 &= ~I2C_CR2_ADD10_M;
		regs->CR2 |= ((addr & 0x7F) << 1) << I2C_CR2_SADD_S;
	}

	for (uint8_t i = 0; i < num_msgs; i = itr) {
		data->current = &msgs[i];
		data->xfer_len = msgs[i].len;

		for (itr = i + 1; itr < num_msgs; itr++) {
			if ((data->current->flags & I2C_MSG_RW_MASK) !=
			    (msgs[itr].flags & I2C_MSG_RW_MASK)) {
				break;
			}
			data->xfer_len += msgs[itr].len;
		}

		if (data->current->flags & I2C_MSG_READ) {
			err = i2c_mik32_msg_read(dev);
		} else {
			err = i2c_mik32_msg_write(dev);
		}

		if (err < 0) {
			i2c_mik32_log_err(data);
			break;
		}
	}

	/* Disable I2C device */
	I2C_CTL0(cfg->reg) &= ~I2C_CTL0_I2CEN;

	k_sem_give(&data->bus_mutex);

	return err;
}

static int i2c_mik32_configure(const struct device *dev,
			      uint32_t dev_config)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	I2C_TypeDef *regs = cfg->regs;
	uint32_t freq;
	int err = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* TIMING можно менять только при PE = 0 */
	regs->CR1 &= ~I2C_CR1_PE_M;

	regs->CR1 &= ~I2C_CR1_ANFOFF_M;
	regs->CR1 |= I2C_ANALOGFILTER_DISABLE << I2C_CR1_ANFOFF_S;

	regs->CR1 &= ~I2C_CR1_DNF_M;
	regs->CR1 |= I2C_CR1_DNF(I2C_DIGITALFILTER_OFF);

	regs->CR1 &= ~I2C_CR1_NOSTRETCH_M;
	regs->CR1 |= I2C_NOSTRETCH_DISABLE << I2C_CR1_NOSTRETCH_S;

	regs->CR2 &= ~I2C_CR2_AUTOEND_M;
	//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;


	(void)clock_control_get_rate(MIK32_CLOCK_CONTROLLER,
				     (clock_control_subsys_t)&cfg->clkid,
				     &pclk1);

	/* i2c clock frequency, us */
	freq = pclk1 / 1000000U;
	if (freq > I2CCLK_MAX) {
		LOG_ERR("I2C max clock freq %u, current is %u\n",
			I2CCLK_MAX, freq);
		err = -ENOTSUP;
		goto error;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_FAST:
#ifdef I2C_FMPCFG
	case I2C_SPEED_FAST_PLUS:
#endif
	case I2C_SPEED_STANDARD:
		if (freq < I2CCLK_MIN) {
			LOG_ERR("I2C standard-mode min clock freq %u, current is %u\n",
				I2CCLK_MIN, freq);
			err = -ENOTSUP;
			goto error;
		}

		/* Standard-mode risetime maximum value: 1000ns */
		regs->TIMINGR = I2C_TIMINGR_SCLDEL(15) | I2C_TIMINGR_SDADEL(15) | I2C_TIMINGR_SCLL(15) | I2C_TIMINGR_SCLH(15) | I2C_TIMINGR_PRESC(5);
		break;
	default:
		err = -EINVAL;
		goto error;
	}

	data->dev_config = dev_config;
error:
	k_sem_give(&data->bus_mutex);

	return err;
}

static DEVICE_API(i2c, i2c_mik32_driver_api) = {
	.configure = i2c_mik32_configure,
	.transfer = i2c_mik32_transfer,
#ifdef CONFIG_I2C_RTIO
	.iodev_submit = i2c_iodev_submit_fallback,
#endif
};

static int i2c_mik32_init(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int err;

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Mutex semaphore to protect the i2c api in multi-thread env. */
	k_sem_init(&data->bus_mutex, 1, 1);

	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

	(void)clock_control_on(MIK32_CLOCK_CONTROLLER,
			       (clock_control_subsys_t)&cfg->clkid);

	cfg->irq_cfg_func();

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	i2c_mik32_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}

#define I2C_MIK32_INIT(idx)							\
	PINCTRL_DT_IDX_DEFINE(idx);						\
	static void i2c_mik32_irq_cfg_func_##idx(void)				\
	{									\
		IRQ_CONNECT(DT_IDX_IRQ_BY_NAME(idx, event, irq),		\
			    DT_IDX_IRQ_BY_NAME(idx, event, priority),		\
			    i2c_mik32_event_isr,				\
			    DEVICE_DT_IDX_GET(idx),				\
			    0);							\
		irq_enable(DT_IDX_IRQ_BY_NAME(idx, event, irq));		\
	}									\
	static struct i2c_mik32_data i2c_mik32_data_##idx;			\
	const static struct i2c_mik32_config i2c_mik32_cfg_##idx = {		\
		.regs = (I2C_TypeDef *)DT_IDX_REG_ADDR(idx),			\
		.bitrate = DT_IDX_PROP(idx, clock_frequency),			\
		.clkid = DT_IDX_CLOCKS_CELL(idx, id),				\
		.pcfg = PINCTRL_DT_IDX_DEV_CONFIG_GET(idx),			\
		.irq_cfg_func = i2c_mik32_irq_cfg_func_##idx,			\
	};									\
	I2C_DEVICE_DT_IDX_DEFINE(idx,						\
				  i2c_mik32_init, NULL,				\
				  &i2c_mik32_data_##idx, &i2c_mik32_cfg_##idx,	\
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,	\
				  &i2c_mik32_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(I2C_MIK32_INIT)
