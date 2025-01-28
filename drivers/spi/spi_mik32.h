#ifndef __MIK32_SPI_H
#define __MIK32_SPI_H

#define SPI_CONFIG        	0X00
#define SPI_STATUS        	0X04
#define SPI_IEN           	0X08 
#define SPI_IDIS          	0X0C 
#define SPI_IMASK         	0X10 
#define SPI_ENABLE        	0X14
#define SPI_DELAY         	0X18
#define SPI_TXD           	0X1C
#define SPI_RXD           	0X20
#define SPI_SIC           	0X24
#define SPI_THRESHOLD     	0X28
#define SPI_MODULE_ID     	0XFC


#define SPI_CONFIG_MANUAL_CS_S		              14
#define SPI_CONFIG_MANUAL_CS_M		              (1 << SPI_CONFIG_MANUAL_CS_S)
//
#define SPI_CONFIG_CS_S				                  10
#define SPI_CONFIG_CS_M				                  (0XF << SPI_CONFIG_CS_S)
#define SPI_CONFIG_CS_0_M			                  (0XE << SPI_CONFIG_CS_S)
#define SPI_CONFIG_CS_1_M			                  (0XD << SPI_CONFIG_CS_S)
#define SPI_CONFIG_CS_2_M			                  (0XB << SPI_CONFIG_CS_S)
#define SPI_CONFIG_CS_3_M			                  (0X7 << SPI_CONFIG_CS_S)
#define SPI_CONFIG_CS_NONE_M		                (0XF << SPI_CONFIG_CS_S)
//
#define SPI_CONFIG_PERI_SEL_S		                9
#define SPI_CONFIG_PERI_SEL_M		                (1 << SPI_CONFIG_PERI_SEL_S)
#define SPI_CONFIG_REF_CLK_S		                8
#define SPI_CONFIG_REF_CLK_M		                (1 << SPI_CONFIG_REF_CLK_S)
//
#define SPI_CONFIG_BAUD_RATE_DIV_S		          3
#define SPI_CONFIG_BAUD_RATE_DIV_M		          (0X7 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_2_M	          (0X0 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_4_M	          (0X1 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_8_M	          (0X2 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_16_M	          (0X3 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_32_M	          (0X4 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_64_M	          (0X5 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_128_M	        (0X6 << SPI_CONFIG_BAUD_RATE_DIV_S)
#define SPI_CONFIG_BAUD_RATE_DIV_256_M	        (0X7 << SPI_CONFIG_BAUD_RATE_DIV_S)
//
#define SPI_MAXIMUM_BAUD_RATE_DIV		            256
//
#define SPI_CONFIG_CLK_PH_S				              2
#define SPI_CONFIG_CLK_PH_M				              (1 << SPI_CONFIG_CLK_PH_S)
#define SPI_CONFIG_CLK_POL_S			              1
#define SPI_CONFIG_CLK_POL_M			              (1 << SPI_CONFIG_CLK_POL_S)
#define SPI_CONFIG_MODE_SEL_S			              0
#define SPI_CONFIG_MODE_SEL_M			              (1 << SPI_CONFIG_MODE_SEL_S)
#define SPI_CONFIG_MASTER_M				              (1 << SPI_CONFIG_MODE_SEL_S)
#define SPI_CONFIG_SLAVE_M				              (0 << SPI_CONFIG_MODE_SEL_S)


#define SPI_ENABLE_CLEAR_RX_FIFO_S              3
#define SPI_ENABLE_CLEAR_RX_FIFO_M              (1 << SPI_ENABLE_CLEAR_RX_FIFO_S)
#define SPI_ENABLE_CLEAR_TX_FIFO_S              2
#define SPI_ENABLE_CLEAR_TX_FIFO_M              (1 << SPI_ENABLE_CLEAR_TX_FIFO_S)
#define SPI_ENABLE_S		                        0
#define SPI_ENABLE_M		                        (1 << SPI_ENABLE_S)


#define SPI_DELAY_BTWN_S                        16
#define SPI_DELAY_BTWN_M                        (0XFF << SPI_DELAY_BTWN_S)
#define SPI_DELAY_BTWN(V)                       (((V) << SPI_DELAY_BTWN_S) & SPI_DELAY_BTWN_M)
#define SPI_DELAY_AFTER_S                       8
#define SPI_DELAY_AFTER_M                       (0XFF << SPI_DELAY_AFTER_S)
#define SPI_DELAY_AFTER(V)                      (((V) << SPI_DELAY_AFTER_S) & SPI_DELAY_AFTER_M)
#define SPI_DELAY_INIT_S                        0
#define SPI_DELAY_INIT_M                        (0XFF << SPI_DELAY_INIT_S)
#define SPI_DELAY_INIT(V)                       (((V) << SPI_DELAY_INIT_S) & SPI_DELAY_INIT_M)


#define SPI_INT_STATUS_SPI_ACTIVE_S             15
#define SPI_INT_STATUS_SPI_ACTIVE_M             (1 << SPI_INT_STATUS_SPI_ACTIVE_S)
#define SPI_INT_STATUS_TX_FIFO_UNDERFLOW_S      6
#define SPI_INT_STATUS_TX_FIFO_UNDERFLOW_M      (1 << SPI_INT_STATUS_TX_FIFO_UNDERFLOW_S)
#define SPI_INT_STATUS_RX_OVERFLOW_S		        0
#define SPI_INT_STATUS_RX_OVERFLOW_M		        (1 << SPI_INT_STATUS_RX_OVERFLOW_S)
#define SPI_INT_STATUS_MODE_FAIL_S			        1
#define SPI_INT_STATUS_MODE_FAIL_M			        (1 << SPI_INT_STATUS_MODE_FAIL_S)
#define SPI_INT_STATUS_TX_FIFO_NOT_FULL_S	      2
#define SPI_INT_STATUS_TX_FIFO_NOT_FULL_M	      (1 << SPI_INT_STATUS_TX_FIFO_NOT_FULL_S)
#define SPI_INT_STATUS_TX_FIFO_FULL_S		        3
#define SPI_INT_STATUS_TX_FIFO_FULL_M		        (1 << SPI_INT_STATUS_TX_FIFO_FULL_S)
#define SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_S	    4
#define SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_M	    (1 << SPI_INT_STATUS_RX_FIFO_NOT_EMPTY_S)
#define SPI_INT_STATUS_RX_FIFO_FULL_S		        5
#define SPI_INT_STATUS_RX_FIFO_FULL_M		        (1 << SPI_INT_STATUS_RX_FIFO_FULL_S)

#define SPI_INT_STATUS_ERR_M (SPI_INT_STATUS_TX_FIFO_UNDERFLOW_M | SPI_INT_STATUS_RX_OVERFLOW_M | SPI_INT_STATUS_MODE_FAIL_M)

#include <stdint.h> 
typedef struct
{
	volatile uint32_t CONFIG;             /* Offset: 0x000 (R/W)    */
	volatile uint32_t INT_STATUS;         /* Offset: 0x004 (R/RC)  */
	volatile uint32_t INT_ENABLE;         /* Offset: 0x008 (WO) */
	volatile uint32_t INT_DISABLE;        /* Offset: 0x00C (WO)  */
	volatile uint32_t INT_MASK;           /* Offset: 0x010 (R)  */
	volatile uint32_t ENABLE;             /* Offset: 0x014 (R/W)  */
	volatile uint32_t DELAY;              /* Offset: 0x018 (R/W)  */
	volatile uint32_t TXDATA;             /* Offset: 0x01C (WO)  */
	volatile uint32_t RXDATA;             /* Offset: 0x020 (RO)  */
	volatile uint32_t SIC;                /* Offset: 0x024 (R/W)  Slave_Idle_Count */
	volatile uint32_t TX_THR;             /* Offset: 0x028 (R/W)  TX threshold */
	volatile uint32_t reserved[0X34];     /* Empty array to fill the space*/
	volatile uint32_t ID;                 /* Offset: 0x0FC (RO)  Module ID 0x01090100 */
} SPI_TypeDef;

/** Значение timeout по умолчанию. */
#define SPI_TIMEOUT_DEFAULT 1000000

/* Ошибки SPI */
#define HAL_SPI_ERROR_NONE  0b00000000  /**< Значение при отсутствии ошибок. */
#define HAL_SPI_ERROR_MODF  0b00000001  /**< Маска для ошибки MODE_FAIL - напряжение на выводе n_ss_in не соответствую режиму работы SPI. */
#define HAL_SPI_ERROR_OVR   0b00000010  /**< Маска для ошибки RX_OVERFLOW - прерывание при переполнении RX_FIFO. */

/* Выбор ведомых устройств. */
#define SPI_CS_NONE 0b1111      /**< Ведомое устройство не выбрано. */
#define SPI_CS_0    0b1110      /**< Ведомое устройство 1. */
#define SPI_CS_1    0b1101      /**< Ведомое устройство 2. */
#define SPI_CS_2    0b1011      /**< Ведомое устройство 3. */
#define SPI_CS_3    0b0111      /**< Ведомое устройство 4. */

/* Коэффициент деления частоты spi_ref_clk. */
#define SPI_BAUDRATE_DIV4   0b001   /**< Коэффициент деления частоты spi_ref_clk - 4. */
#define SPI_BAUDRATE_DIV8   0b010   /**< Коэффициент деления частоты spi_ref_clk - 8. */
#define SPI_BAUDRATE_DIV16  0b011   /**< Коэффициент деления частоты spi_ref_clk - 16. */
#define SPI_BAUDRATE_DIV32  0b100   /**< Коэффициент деления частоты spi_ref_clk - 32. */
#define SPI_BAUDRATE_DIV64  0b101   /**< Коэффициент деления частоты spi_ref_clk - 64. */
#define SPI_BAUDRATE_DIV128 0b110   /**< Коэффициент деления частоты spi_ref_clk - 128. */
#define SPI_BAUDRATE_DIV256 0b111   /**< Коэффициент деления частоты spi_ref_clk - 256. */

/* Режим управления сигналом выбора ведомого CS. */
#define SPI_MANUALCS_OFF    0   /**< Автоматический режим. */
#define SPI_MANUALCS_ON     1   /**< Ручной режим. */

/* Настройки фазы тактового сигнала. */
#define SPI_PHASE_OFF   0   /**< Тактовая частота SPI активна вне слова */
#define SPI_PHASE_ON    1   /**< Тактовая частота SPI неактивна вне слова */

/* Настройки полярности тактового сигнала вне слова. */
#define SPI_POLARITY_LOW    0  /**< Тактовый сигнал вне слова удерживается на низком уровне. */
#define SPI_POLARITY_HIGH   1  /**< Тактовый сигнал вне слова удерживается на высоком уровне. */

/* Использование внешнего декодера. */
#define SPI_DECODER_NONE 0  /**< Внешний декодер не используется. Выбор только 1 из 4 ведомых устройств. */
#define SPI_DECODER_USE  1  /**< Используется внешний декодер. */

/* Длина передаваемой посылки. */
#define SPI_DATASIZE_8BITS  0  /**< Длина передаваемой посылки - 8 бит. */
#define SPI_DATASIZE_16BITS 1  /**< Длина передаваемой посылки - 16 бит. */
#define SPI_DATASIZE_24BITS 2  /**< Длина передаваемой посылки - 24 бит. */
#define SPI_DATASIZE_32BITS 3  /**< Длина передаваемой посылки - 32 бит. */

/* Значения по умолчанию порогового значения TX_FIFO. */
#define SPI_THRESHOLD_DEFAULT 4 /* Значение Threshold_of_TX_FIFO по умолчанию*/

/* Прерывания. */
#define TX_FIFO_UNDERFLOW   6   /**< Регистр TX FIFO опустошен. */
#define RX_FIFO_FULL        5   /**< Регистр RX_FIFO заполнен. */
#define RX_FIFO_NOT_EMPTY   4   /**< Регистр RX_FIFO не пустой. */
#define TX_FIFO_FULL        3   /**< Регистр TX_FIFO заполнен. */
#define TX_FIFO_NOT_FULL    2   /**< Регистр TX_FIFO не заполнен. */
#define MODE_FAIL           1   /**< Напряжение на выводе n_ss_in не соответствую режиму работы SPI. */
#define RX_OVERFLOW         0   /**< Прерывание при переполнении RX_FIFO, значение сбрасывается при чтении. */

#endif/*__MIK32_SPI_H*/
