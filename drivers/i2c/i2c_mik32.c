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
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/i2c/i2c.h>

#include <zephyr/soc/mik32_epic.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_mik32, CONFIG_I2C_LOG_LEVEL);

#include "i2c_mik32.h"

struct i2c_mik32_config {
	I2C_TypeDef * regs;
	const struct device *clock_dev;
	uint32_t bitrate;
	uint16_t clkid;
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
#ifdef CONFIG_I2C_TARGET
	bool master_active;
  struct i2c_target_config *slave_cfg;
  bool slave_attached;
#endif
};

#if defined(CONFIG_I2C_TARGET)
static void mik32_i2c_slave_isr(const struct device *dev)
{
	const struct i2c_mik32_config *cfg = dev->config;
	struct i2c_mik32_data *data = dev->data;
	I2C_TypeDef *i2c = cfg->i2c;
	const struct i2c_target_callbacks *slave_cb = data->slave_cfg->callbacks;
}

/* Attach and start I2C as slave */
int i2c_mik32_target_register(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_stm32_config *cfg = dev->config;
	struct i2c_stm32_data *data = dev->data;
	I2C_TypeDef *i2c = cfg->i2c;
	uint32_t bitrate_cfg;
	int ret;

	if (!config) {
		return -EINVAL;
	}

	if (data->slave_attached) {
		return -EBUSY;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	ret = i2c_mik32_runtime_configure(dev, bitrate_cfg);
	if (ret < 0) {
		LOG_ERR("i2c: failed to initialize");
		return ret;
	}

	data->slave_cfg = config;

	//I2C_Enable(i2c);

	if (data->slave_cfg->flags == I2C_TARGET_FLAGS_ADDR_10_BITS)	{
		return -ENOTSUP;
	}
	mik32_i2c_set_own_addr1(i2c, config->address << 1U);
	data->slave_attached = true;

	LOG_DBG("i2c: target registered");

	mik32_i2c_enable_transfer_interrupts(dev);
	//I2C_AcknowledgeNextData(i2c, LL_I2C_ACK);

	return 0;
}

int i2c_mik32_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	const struct i2c_stm32_config *cfg = dev->config;
	struct i2c_stm32_data *data = dev->data;
	I2C_TypeDef *i2c = cfg->i2c;

	if (!data->slave_attached) {
		return -EINVAL;
	}

	if (data->master_active) {
		return -EBUSY;
	}

	//mik32_i2c_disable_transfer_interrupts(dev);

	/* if (!data->smbalert_active) { */
	/* 	I2C_Disable(i2c); */
	/* } */

	data->slave_attached = false;

	LOG_DBG("i2c: slave unregistered");

	return 0;
}
#endif /* defined(CONFIG_I2C_TARGET) */

static inline uint32_t i2c_map_dt_bitrate(uint32_t bitrate)
{
	switch (bitrate) {
	case I2C_BITRATE_STANDARD:
		return I2C_SPEED_STANDARD << I2C_SPEED_SHIFT;
	case I2C_BITRATE_FAST:
		return I2C_SPEED_FAST << I2C_SPEED_SHIFT;
	case I2C_BITRATE_FAST_PLUS:
		return I2C_SPEED_FAST_PLUS << I2C_SPEED_SHIFT;
	case I2C_BITRATE_HIGH:
		return I2C_SPEED_HIGH << I2C_SPEED_SHIFT;
	case I2C_BITRATE_ULTRA:
		return I2C_SPEED_ULTRA << I2C_SPEED_SHIFT;
	}

	LOG_ERR("Invalid I2C bit rate value");

	return 0;
}

static inline void i2c_mik32_enable_tx_interrupts(const struct i2c_mik32_config  *cfg)
{
	cfg->regs->CR1 |= I2C_CR1_ERRIE_M | I2C_CR1_TCIE_M | I2C_CR1_NACKIE_M | I2C_CR1_TXIE_M;
}

static inline void i2c_mik32_enable_rx_interrupts(const struct i2c_mik32_config  *cfg)
{
	cfg->regs->CR1 |= I2C_CR1_ERRIE_M | I2C_CR1_TCIE_M | I2C_CR1_ADDRIE_M | I2C_CR1_RXIE_M;
}

static inline void i2c_mik32_disable_interrupts(const struct i2c_mik32_config  *cfg)
{
	cfg->regs->CR1 &= ~I2C_INTMASK;
}

static void i2c_mik32_isr(const void *param)
{
  const struct device *dev = (const struct device *)param;
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;

	uint32_t int_mask = cfg->regs->CR1 & I2C_INTMASK; /* разрешенные прерывания  */
	uint32_t status = cfg->regs->ISR; /* Флаги */

	if ((status & I2C_ISR_ADDR_M) && (int_mask & I2C_CR1_ADDRIE_M)) {
		if (cfg->regs->CR1 & I2C_CR1_SBC_M) {
			cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
			cfg->regs->CR2 |= I2C_CR2_NBYTES(0x1);
		}

		/* Сброс флага ADDR */
		cfg->regs->ICR |= I2C_ICR_ADDRCF_M;
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
		cfg->regs->CR1 &= ~I2C_CR1_PE_M;
		cfg->regs->CR1 |= I2C_CR1_PE_M;
	}

	if ((status & I2C_ISR_NACKF_M) && (int_mask & I2C_CR1_NACKIE_M)) {
		/* Выключить все прерывания I2C */
		i2c_mik32_disable_interrupts(cfg);
		/* Сброс I2C */
		data->errs = I2C_ERROR_NACK;
		cfg->regs->CR1 &= ~I2C_CR1_PE_M;
		cfg->regs->CR1 |= I2C_CR1_PE_M;
	}

	if ((status & I2C_ISR_STOPF_M) && (int_mask & I2C_CR1_STOPIE_M)) {
		/* Сброс содержимого TXDR */
		cfg->regs->ISR |= I2C_ISR_TXE_M;
		/* Сброс флага детектирования STOP на шине */
		cfg->regs->ICR |= I2C_ICR_STOPCF_M;

		cfg->regs->CR2 |= I2C_CR2_STOP_M;
		k_sem_give(&data->sync_sem);
	}

	if ((status & I2C_ISR_TXIS_M) && (int_mask & I2C_CR1_TXIE_M)) {
		data->xfered_len++;
		if ((data->xfered_len > data->xfer_len) && (data->master_active == false))
		{
			cfg->regs->CR1 &= ~I2C_CR1_PE_M;
			cfg->regs->CR1 |= I2C_CR1_PE_M;

			cfg->regs->CR2 |= I2C_CR2_STOP_M;
			k_sem_give(&data->sync_sem);
		}
		else
		{
			cfg->regs->TXDR = *((uint8_t *)data->current->buf);
			data->current->buf++;
			if (data->xfered_len == data->xfer_len)
			{
				cfg->regs->CR2 |= I2C_CR2_STOP_M;
				k_sem_give(&data->sync_sem);
			}
		}
	}

	if ((status & I2C_ISR_RXNE_M) && (int_mask & I2C_CR1_RXIE_M)) {
		*((uint8_t *)data->current->buf) = (uint8_t)cfg->regs->RXDR;

		if (cfg->regs->CR1 & I2C_CR1_SBC_M)
		{
			cfg->regs->CR2 &= ~I2C_CR2_NACK_M; /* Формирование ACK */
		}

		data->current->buf++;
		data->xfered_len++;
		if (data->xfered_len == data->xfer_len)
		{
			cfg->regs->CR2 |= I2C_CR2_STOP_M;
			k_sem_give(&data->sync_sem);
		}
	}

	if ((status & I2C_ISR_TCR_M) && (int_mask & I2C_CR1_TCIE_M)) {
		if (cfg->regs->CR1 & I2C_CR1_SBC_M)
		{
			*((uint8_t *)data->current->buf) = (uint8_t)cfg->regs->RXDR;
			data->current->buf++;
			data->xfered_len++;

			cfg->regs->CR2 &= ~I2C_CR2_NACK_M; /* Формирование ACK */
			/* Выключить все прерывания I2C */
			i2c_mik32_disable_interrupts(cfg);
			/* Сброс I2C */
			cfg->regs->CR1 &= ~I2C_CR1_PE_M;
			cfg->regs->CR1 |= I2C_CR1_PE_M;

			if (data->xfered_len < data->xfer_len)
			{
				cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
				cfg->regs->CR2 |= I2C_CR2_NBYTES(0x1);
			}
			else
			{
				cfg->regs->CR1 &= ~(I2C_CR1_TXIE_M);
				cfg->regs->CR2 |= I2C_CR2_STOP_M;
				k_sem_give(&data->sync_sem);
			}
		}
		else
		{
			cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
			/* Подготовка перед отправкой */
		
			if ((data->xfer_len - data->xfered_len) <= I2C_NBYTE_MAX)
			{
				cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
				cfg->regs->CR2 |= I2C_CR2_NBYTES(data->xfer_len - data->xfered_len);
				cfg->regs->CR2 &= ~I2C_CR2_RELOAD_M;
				cfg->regs->CR2 &= ~I2C_CR2_AUTOEND_M;
				//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;
			}
			else /* DataSize > 255 */
			{
				cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
				cfg->regs->CR2 |= I2C_CR2_NBYTES(I2C_NBYTE_MAX);
				/* При RELOAD = 1 AUTOEND игнорируется */
				cfg->regs->CR2 |= I2C_CR2_RELOAD_M;
			}
		}
	}

	if ((status & I2C_ISR_TC_M) && (int_mask & I2C_CR1_TCIE_M)) {
		cfg->regs->CR1 &= ~(I2C_CR1_TXIE_M);
		cfg->regs->CR2 |= I2C_CR2_STOP_M;
		k_sem_give(&data->sync_sem);
	}

	if (data->errs != 0U) {
		/* Enter stop condition */
		cfg->regs->CR2 |= I2C_CR2_STOP_M;

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

	k_sem_reset(&data->sync_sem);

	data->errs = 0U;
	data->is_restart = false;
	data->xfered_len = 0;

	i2c_mik32_disable_interrupts(cfg);
	if ((data->xfer_len - data->xfered_len) <= I2C_NBYTE_MAX) {
		cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
		cfg->regs->CR2 |= I2C_CR2_NBYTES(data->xfer_len - data->xfered_len);
		cfg->regs->CR2 &= ~I2C_CR2_RELOAD_M;
		cfg->regs->CR2 &= ~I2C_CR2_AUTOEND_M;
		//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;
	} else {
		cfg->regs->CR2 &= ~I2C_CR2_NBYTES_M;
		cfg->regs->CR2 |= I2C_CR2_NBYTES(I2C_NBYTE_MAX);
		/* При RELOAD = 1 AUTOEND игнорируется */
		cfg->regs->CR2 |= I2C_CR2_RELOAD_M;
	}

	if (data->current->flags & I2C_MSG_READ) {
		cfg->regs->CR2 |= I2C_CR2_RD_WRN_M;
		i2c_mik32_enable_rx_interrupts(cfg);
	} else {
		cfg->regs->CR2 &= ~I2C_CR2_RD_WRN_M;
		i2c_mik32_enable_tx_interrupts(cfg);
	}

	/* Enter start condition */
	cfg->regs->CR2 |= I2C_CR2_START_M;
}

static int i2c_mik32_xfer_end(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;

	i2c_mik32_disable_interrupts(cfg);

	/* Wait for stop condition is done. */
	while (cfg->regs->ISR & I2C_ISR_BUSY_M) {
		/* NOP */
	}

	if (data->errs) {
		return -EIO;
	}

	return 0;
}

static int i2c_mik32_msg_rw(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;

	if (cfg->regs->ISR & I2C_ISR_BUSY_M) {
		data->errs = I2C_ERROR_STOP;
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
	cfg->regs->CR1 |= I2C_CR1_PE_M;

	if (data->dev_config & I2C_ADDR_10_BITS) {
		cfg->regs->CR2 |= I2C_CR2_ADD10_M;
		cfg->regs->CR2 &= ~I2C_CR2_HEAD10R_M; /* ведущий отправляет полную последовательность для чтения для 10 битного адреса */
		cfg->regs->CR2 |= (addr & 0x3FF) << I2C_CR2_SADD_S;
	} else {
		cfg->regs->CR2 &= ~I2C_CR2_ADD10_M;
		cfg->regs->CR2 |= ((addr & 0x7F) << 1) << I2C_CR2_SADD_S;
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
			err = i2c_mik32_msg_rw(dev);
		} else {
			err = i2c_mik32_msg_rw(dev);
		}

		if (err < 0) {
			i2c_mik32_log_err(data);
			break;
		}
	}

	/* Disable I2C device */
	cfg->regs->CR1 &= ~I2C_CR1_PE_M;

	k_sem_give(&data->bus_mutex);

	return err;
}

static int i2c_mik32_configure(const struct device *dev,
			      uint32_t dev_config)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t)&cfg->clkid;
	uint32_t clock_rate;
	int err = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* TIMING можно менять только при PE = 0 */
	cfg->regs->CR1 &= ~I2C_CR1_PE_M;

	cfg->regs->CR1 &= ~I2C_CR1_ANFOFF_M;
	cfg->regs->CR1 |= I2C_ANALOGFILTER_DISABLE << I2C_CR1_ANFOFF_S;

	cfg->regs->CR1 &= ~I2C_CR1_DNF_M;
	cfg->regs->CR1 |= I2C_CR1_DNF(I2C_DIGITALFILTER_OFF);

	cfg->regs->CR1 &= ~I2C_CR1_NOSTRETCH_M;
	cfg->regs->CR1 |= I2C_NOSTRETCH_DISABLE << I2C_CR1_NOSTRETCH_S;

	cfg->regs->CR2 &= ~I2C_CR2_AUTOEND_M;
	//regs->CR2 |= I2C_AUTOEND_DISABLE << I2C_CR2_AUTOEND_S;

	err = clock_control_get_rate(cfg->clock_dev, clock_sys, &clock_rate);
	if (err != 0) {
		return err;
	}

	if (I2C_SPEED_GET(dev_config) != I2C_SPEED_STANDARD) {
		LOG_WRN("Only standard speed is implemented");
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_FAST:
#ifdef I2C_FMPCFG
	case I2C_SPEED_FAST_PLUS:
#endif
	case I2C_SPEED_STANDARD:
		/* Standard-mode risetime maximum value: 1000ns */
		cfg->regs->TIMINGR = I2C_TIMINGR_SCLDEL(15) | I2C_TIMINGR_SDADEL(15) | I2C_TIMINGR_SCLL(15) | I2C_TIMINGR_SCLH(15) | I2C_TIMINGR_PRESC(5);
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
#if defined(CONFIG_I2C_TARGET)
	.target_register = i2c_mik32_target_register,
	.target_unregister = i2c_mik32_target_unregister,
#endif
};

static int i2c_mik32_init(const struct device *dev)
{
	struct i2c_mik32_data *data = dev->data;
	const struct i2c_mik32_config *cfg = dev->config;
	clock_control_subsys_t clock_sys = (clock_control_subsys_t)&cfg->clkid;
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

	(void)clock_control_on(cfg->clock_dev, clock_sys);

	cfg->irq_cfg_func();

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);

	i2c_mik32_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}

#define I2C_MIK32_INIT(idx)                                             \
	PINCTRL_DT_INST_DEFINE(idx);                                          \
	static void i2c_mik32_irq_cfg_func_##idx(void)                        \
	{                                                                     \
    mik32_irq_connect_dynamic(DT_INST_IRQN(idx), 0, &i2c_mik32_isr, DEVICE_DT_INST_GET(idx), 0); \
		irq_enable(DT_INST_IRQN(idx));                                      \
	}                                                                     \
	static struct i2c_mik32_data i2c_mik32_data_##idx;                    \
	const static struct i2c_mik32_config i2c_mik32_cfg_##idx = {          \
		.regs = (I2C_TypeDef *)DT_INST_REG_ADDR(idx),                       \
		.bitrate = DT_INST_PROP(idx, clock_frequency),                      \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),               \
		.clkid = DT_INST_CLOCKS_CELL(idx, id),                              \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                        \
		.irq_cfg_func = i2c_mik32_irq_cfg_func_##idx,                       \
	};                                                                    \
	I2C_DEVICE_DT_INST_DEFINE(idx,                                        \
                            i2c_mik32_init, NULL,                       \
                            &i2c_mik32_data_##idx, &i2c_mik32_cfg_##idx, \
                            POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
                            &i2c_mik32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_MIK32_INIT)
