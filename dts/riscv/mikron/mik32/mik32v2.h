/*
 * SPDX-License-Identifier: Apache 2.0
 */

#include <zephyr/dt-bindings/pinctrl/mik32-af.h>
#include <zephyr/dt-bindings/pinctrl/mik32-pincfg.h>
//#include <soc/mikron/mik32/pinctrl_soc.h>
//#include <soc/mikron/mik32/pinctrl_soc.h>

/* I2C1_SDA */
#define I2C1_SDA_PB12 \
	MIK32_PINMUX_AF('B', 12, AF1)

/* USART1_RTS */
#define USART1_RTS_PB11 \
	(MIK32_PINMUX_AF('B', 11, AF1) | MIK32_PINMUX_DIR_OUT)

/* USART1_CTS */
#define USART1_CTS_PB10 \
	(MIK32_PINMUX_AF('B', 10, AF1) | MIK32_PINMUX_DIR_IN)

/* USART1_TX */
#define USART1_TX_PB9 \
	(MIK32_PINMUX_AF('B', 9, AF1) | MIK32_PINMUX_DIR_OUT)

/* USART1_RX */
#define USART1_RX_PB8 \
	(MIK32_PINMUX_AF('B', 8, AF1) | MIK32_PINMUX_DIR_IN)

/* SPI1_NSS3 */
#define SPI1_NSS3_PB7 \
	(MIK32_PINMUX_AF('B', 7, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_NSS2 */
#define SPI1_NSS2_PB6 \
	(MIK32_PINMUX_AF('B', 6, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_NSS1 */
#define SPI1_NSS1_PB5 \
	(MIK32_PINMUX_AF('B', 5, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_NSS0 */
#define SPI1_NSS0_PB4 \
	(MIK32_PINMUX_AF('B', 4, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_NSSIN */
#define SPI1_NSSIN_PB3 \
	(MIK32_PINMUX_AF('B', 3, AF1) | MIK32_PINMUX_DIR_IN | MIK32_PINMUX_PUPD_PU)

/* SPI1_CLK */
#define SPI1_CLK_PB2 \
	(MIK32_PINMUX_AF('B', 2, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_MOSI */
#define SPI1_MOSI_PB1 \
	(MIK32_PINMUX_AF('B', 1, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI1_MISO */
#define SPI1_MISO_PB0 \
	(MIK32_PINMUX_AF('B', 0, AF1) | MIK32_PINMUX_DIR_IN)

/* SPI0_NSS3 */
#define SPI0_NSS3_PC6 \
	(MIK32_PINMUX_AF('C', 6, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPIFI_DATA3 */
#define SPIFI_DATA3_PC5 \
	MIK32_PINMUX_AF('C', 5, AF1)

/* SPIFI_DATA2 */
#define SPIFI_DATA2_PC4 \
	MIK32_PINMUX_AF('C', 4, AF1)

/* SPIFI_DATA1 */
#define SPIFI_DATA1_PC3 \
	MIK32_PINMUX_AF('C', 3, AF1)

/* SPIFI_DATA0 */
#define SPIFI_DATA0_PC2 \
	MIK32_PINMUX_AF('C', 2, AF1)

/* SPIFI_CS */
#define SPIFI_CS_PC1 \
	(MIK32_PINMUX_AF('C', 1, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPIFI_SCLK */
#define SPIFI_SCLK_PC0 \
	(MIK32_PINMUX_AF('C', 0, AF1) | MIK32_PINMUX_DIR_OUT)

/* I2C0_SCL */
#define I2C0_SCL_PA10 \
	(MIK32_PINMUX_AF('A', 10, AF1) | MIK32_PINMUX_DIR_OUT)

/* I2C0_SDA */
#define I2C0_SDA_PA9 \
	MIK32_PINMUX_AF('A', 9, AF1)

/* USART0_RTS */
#define USART0_RTS_PA8 \
	(MIK32_PINMUX_AF('A', 8, AF1) | MIK32_PINMUX_DIR_OUT)

/* USART0_CTS */
#define USART0_CTS_PA7 \
	(MIK32_PINMUX_AF('A', 7, AF1) | MIK32_PINMUX_DIR_IN)

/* USART0_TX */
#define USART0_TX_PA6 \
	(MIK32_PINMUX_AF('A', 6, AF1) | MIK32_PINMUX_DIR_OUT)

/* USART0_RX */
#define USART0_RX_PA5 \
	(MIK32_PINMUX_AF('A', 5, AF1) | MIK32_PINMUX_DIR_IN)

/* SPI0_NSS0 */
#define SPI0_NSS0_PA4 \
	(MIK32_PINMUX_AF('A', 4, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI0_NSSIN */
#define SPI0_NSSIN_PA3 \
	(MIK32_PINMUX_AF('A', 3, AF1) | MIK32_PINMUX_DIR_IN | MIK32_PINMUX_PUPD_PU)

/* SPI0_CLK */
#define SPI0_CLK_PA2 \
	(MIK32_PINMUX_AF('A', 2, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI0_MOSI */
#define SPI0_MOSI_PA1 \
	(MIK32_PINMUX_AF('A', 1, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI0_MISO */
#define SPI0_MISO_PA0 \
	(MIK32_PINMUX_AF('A', 0, AF1) | MIK32_PINMUX_DIR_IN)

/* SPI0_NSS2 */
#define SPI0_NSS2_PB15 \
	(MIK32_PINMUX_AF('B', 15, AF1) | MIK32_PINMUX_DIR_OUT)

/* SPI0_NSS1 */
#define SPI0_NSS1_PB14 \
	(MIK32_PINMUX_AF('B', 14, AF1) | MIK32_PINMUX_DIR_OUT)

/* I2C1_SCL */
#define I2C1_SCL_PB13 \
	(MIK32_PINMUX_AF('B', 13, AF1) | MIK32_PINMUX_DIR_OUT)

/* USART0_DDIS */
#define USART0_DDIS_PB6 \
	MIK32_PINMUX_AF('B', 6, AF2)

/* USART0_CK */
#define USART0_CK_PB5 \
	MIK32_PINMUX_AF('B', 5, AF2)

/* USART1_CK */
#define USART1_CK_PC6 \
	MIK32_PINMUX_AF('C', 6, AF2)

/* USART1_RI */
#define USART1_RI_PC3 \
	MIK32_PINMUX_AF('C', 3, AF2)

/* USART1_DSR */
#define USART1_DSR_PC2 \
	MIK32_PINMUX_AF('C', 2, AF2)

/* USART1_DCD */
#define USART1_DCD_PC1 \
	MIK32_PINMUX_AF('C', 1, AF2)

/* USART1_DTR */
#define USART1_DTR_PC0 \
	MIK32_PINMUX_AF('C', 0, AF2)

/* USART0_RI */
#define USART0_RI_PB15 \
	MIK32_PINMUX_AF('B', 15, AF2)

/* USART0_DSR */
#define USART0_DSR_PB14 \
	MIK32_PINMUX_AF('B', 14, AF2)

/* USART0_DCD */
#define USART0_DCD_PB13 \
	MIK32_PINMUX_AF('B', 13, AF2)

/* ADC_IN0 */
#define ADC_IN0_PB5 \
	MIK32_PINMUX_AF('B', 5, ANALOG)

/* ADC_IN1 */
#define ADC_IN1_PB7 \
	MIK32_PINMUX_AF('B', 7, ANALOG)

/* ADC_IN2 */
#define ADC_IN2_PA2 \
	MIK32_PINMUX_AF('A', 2, ANALOG)

/* ADC_IN3 */
#define ADC_IN3_PA4 \
	MIK32_PINMUX_AF('A', 4, ANALOG)

/* ADC_IN4 */
#define ADC_IN4_PA7 \
	MIK32_PINMUX_AF('A', 7, ANALOG)

/* ADC_IN5 */
#define ADC_IN5_PA9 \
	MIK32_PINMUX_AF('A', 9, ANALOG)

/* ADC_IN6 */
#define ADC_IN6_PA11 \
	MIK32_PINMUX_AF('A', 11, ANALOG)

/* ADC_IN7 */
#define ADC_IN7_PA13 \
	MIK32_PINMUX_AF('A', 13, ANALOG)

/* JTCK */
#define JTCK_PA12 \
	MIK32_PINMUX_AF('A', 12, AF1)

/* JTDI */
#define JTDI_PA11 \
	MIK32_PINMUX_AF('A', 11, AF1)

/* JTDO */
#define JTDO_PA15 \
	MIK32_PINMUX_AF('A', 15, AF1)

/* JTMS */
#define JTMS_PA13 \
	MIK32_PINMUX_AF('A', 13, AF1)

/* NJTRST */
#define NJTRST_PA14 \
	MIK32_PINMUX_AF('A', 14, AF1)
