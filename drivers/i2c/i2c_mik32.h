#ifndef __I2C_MIK32_H
#define __I2C_MIK32_H

#define I2C_CR1_PE_S                    0
#define I2C_CR1_PE_M                    (1 << I2C_CR1_PE_S)
#define I2C_CR1_TXIE_S                  1
#define I2C_CR1_TXIE_M                  (1 << I2C_CR1_TXIE_S)
#define I2C_CR1_RXIE_S                  2
#define I2C_CR1_RXIE_M                  (1 << I2C_CR1_RXIE_S)
#define I2C_CR1_ADDRIE_S                3
#define I2C_CR1_ADDRIE_M                (1 << I2C_CR1_ADDRIE_S)
#define I2C_CR1_NACKIE_S                4
#define I2C_CR1_NACKIE_M                (1 << I2C_CR1_NACKIE_S)
#define I2C_CR1_STOPIE_S                5
#define I2C_CR1_STOPIE_M                (1 << I2C_CR1_STOPIE_S)
#define I2C_CR1_TCIE_S                  6
#define I2C_CR1_TCIE_M                  (1 << I2C_CR1_TCIE_S)
#define I2C_CR1_ERRIE_S                 7
#define I2C_CR1_ERRIE_M                 (1 << I2C_CR1_ERRIE_S)
#define I2C_CR1_DNF_S                   8
#define I2C_CR1_DNF_M                   (0xF << I2C_CR1_DNF_S)
#define I2C_CR1_DNF(v)                  (((v) << I2C_CR1_DNF_S) & I2C_CR1_DNF_M)
#define I2C_CR1_ANFOFF_S                12
#define I2C_CR1_ANFOFF_M                (1 << I2C_CR1_ANFOFF_S)
//
#define I2C_CR1_TXDMAEN_S               14
#define I2C_CR1_TXDMAEN_M               (1 << I2C_CR1_TXDMAEN_S)
#define I2C_CR1_RXDMAEN_S               15
#define I2C_CR1_RXDMAEN_M               (1 << I2C_CR1_RXDMAEN_S)
#define I2C_CR1_SBC_S                   16
#define I2C_CR1_SBC_M                   (1 << I2C_CR1_SBC_S)
#define I2C_CR1_NOSTRETCH_S             17
#define I2C_CR1_NOSTRETCH_M             (1 << I2C_CR1_NOSTRETCH_S)
//
#define I2C_CR1_GCEN_S                  19
#define I2C_CR1_GCEN_M                  (1 << I2C_CR1_GCEN_S)


#define I2C_CR2_SADD_S                  0
#define I2C_CR2_SADD_M                  (0x3FF << I2C_CR2_SADD_S)
#define I2C_CR2_SADD(v)                 (((v) << I2C_CR2_SADD_S) &I2C_CR2_SADD_M)
#define I2C_CR2_RD_WRN_S                10    
#define I2C_CR2_RD_WRN_M                (1 << I2C_CR2_RD_WRN_S)
#define I2C_CR2_RD_M                    (1 << I2C_CR2_RD_WRN_S)
#define I2C_CR2_WR_M                    (0 << I2C_CR2_RD_WRN_S)
#define I2C_CR2_ADD10_S                 11
#define I2C_CR2_ADD10_M                 (1 << I2C_CR2_ADD10_S)
#define I2C_CR2_HEAD10R_S               12
#define I2C_CR2_HEAD10R_M               (1 << I2C_CR2_HEAD10R_S)
#define I2C_CR2_START_S                 13
#define I2C_CR2_START_M                 (1 << I2C_CR2_START_S)
#define I2C_CR2_STOP_S                  14
#define I2C_CR2_STOP_M                  (1 << I2C_CR2_STOP_S)
#define I2C_CR2_NACK_S                  15
#define I2C_CR2_NACK_M                  (1 << I2C_CR2_NACK_S)
#define I2C_CR2_NBYTES_S                16  
#define I2C_CR2_NBYTES_M                (0xFF << I2C_CR2_NBYTES_S)
#define I2C_CR2_NBYTES(v)               (((v) << I2C_CR2_NBYTES_S) & I2C_CR2_NBYTES_M)
#define I2C_CR2_RELOAD_S                24
#define I2C_CR2_RELOAD_M                (1 << I2C_CR2_RELOAD_S)
#define I2C_CR2_AUTOEND_S               25
#define I2C_CR2_AUTOEND_M               (1 << I2C_CR2_AUTOEND_S)


#define I2C_OAR1_OA1_S                  0
#define I2C_OAR1_OA1_M                  (0x3FF << I2C_OAR1_OA1_S)
#define I2C_OAR1_OA1MODE_S              10
#define I2C_OAR1_OA1MODE_M              (1 << I2C_OAR1_OA1MODE_S)
//
#define I2C_OAR1_OA1EN_S                15
#define I2C_OAR1_OA1EN_M                (1 << I2C_OAR1_OA1EN_S)


#define I2C_OAR2_OA2_S                  1
#define I2C_OAR2_OA2_M                  (0x7F << I2C_OAR2_OA2_S)
#define I2C_OAR2_OA2MSK_S              8
#define I2C_OAR2_OA2MSK_M              (0x7 << I2C_OAR2_OA2MSK_S)
//
#define I2C_OAR2_OA2EN_S                15
#define I2C_OAR2_OA2EN_M                (1 << I2C_OAR2_OA2EN_S)


#define I2C_TIMINGR_SCLL_S              0
#define I2C_TIMINGR_SCLL_M              (0xFF << I2C_TIMINGR_SCLL_S)
#define I2C_TIMINGR_SCLL(v)             (((v) << I2C_TIMINGR_SCLL_S) & I2C_TIMINGR_SCLL_M)
#define I2C_TIMINGR_SCLH_S              8
#define I2C_TIMINGR_SCLH_M              (0xFF << I2C_TIMINGR_SCLH_S)
#define I2C_TIMINGR_SCLH(v)             (((v) << I2C_TIMINGR_SCLH_S) & I2C_TIMINGR_SCLH_M)
#define I2C_TIMINGR_SDADEL_S            16
#define I2C_TIMINGR_SDADEL_M            (0xF << I2C_TIMINGR_SDADEL_S)
#define I2C_TIMINGR_SDADEL(v)           (((v) << I2C_TIMINGR_SDADEL_S) & I2C_TIMINGR_SDADEL_M)
#define I2C_TIMINGR_SCLDEL_S            20
#define I2C_TIMINGR_SCLDEL_M            (0xF << I2C_TIMINGR_SCLDEL_S)
#define I2C_TIMINGR_SCLDEL(v)           (((v) << I2C_TIMINGR_SCLDEL_S) & I2C_TIMINGR_SCLDEL_M)
#define I2C_TIMINGR_PRESC_S             28
#define I2C_TIMINGR_PRESC_M             (0xF << I2C_TIMINGR_PRESC_S)
#define I2C_TIMINGR_PRESC(v)            (((v) << I2C_TIMINGR_PRESC_S) & I2C_TIMINGR_PRESC_M)


#define I2C_ISR_TXE_S                   0
#define I2C_ISR_TXE_M                   (1 << I2C_ISR_TXE_S)
#define I2C_ISR_TXIS_S                  1
#define I2C_ISR_TXIS_M                  (1 << I2C_ISR_TXIS_S)
#define I2C_ISR_RXNE_S                  2
#define I2C_ISR_RXNE_M                  (1 << I2C_ISR_RXNE_S)
#define I2C_ISR_ADDR_S                  3
#define I2C_ISR_ADDR_M                  (1 << I2C_ISR_ADDR_S)
#define I2C_ISR_NACKF_S                 4
#define I2C_ISR_NACKF_M                 (1 << I2C_ISR_NACKF_S)
#define I2C_ISR_STOPF_S                 5
#define I2C_ISR_STOPF_M                 (1 << I2C_ISR_STOPF_S)
#define I2C_ISR_TC_S                    6
#define I2C_ISR_TC_M                    (1 << I2C_ISR_TC_S)
#define I2C_ISR_TCR_S                   7
#define I2C_ISR_TCR_M                   (1 << I2C_ISR_TCR_S)
#define I2C_ISR_BERR_S                  8
#define I2C_ISR_BERR_M                  (1 << I2C_ISR_BERR_S)
#define I2C_ISR_ARLO_S                  9
#define I2C_ISR_ARLO_M                  (1 << I2C_ISR_ARLO_S)
#define I2C_ISR_OVR_S                   10
#define I2C_ISR_OVR_M                   (1 << I2C_ISR_OVR_S)
//
#define I2C_ISR_BUSY_S                  15
#define I2C_ISR_BUSY_M                  (1 << I2C_ISR_BUSY_S)
#define I2C_ISR_DIR_S                   16
#define I2C_ISR_DIR_M                   (1 << I2C_ISR_DIR_S)
#define I2C_ISR_ADDCODE_S               17
#define I2C_ISR_ADDCODE_M               (0x7F << I2C_ISR_ADDCODE_S)


#define I2C_ICR_ADDRCF_S                3
#define I2C_ICR_ADDRCF_M                (1 << I2C_ICR_ADDRCF_S)
#define I2C_ICR_NACKCF_S                4
#define I2C_ICR_NACKCF_M                (1 << I2C_ICR_NACKCF_S)
#define I2C_ICR_STOPCF_S                5
#define I2C_ICR_STOPCF_M                (1 << I2C_ICR_STOPCF_S)
//
#define I2C_ICR_BERRCF_S                8
#define I2C_ICR_BERRCF_M                (1 << I2C_ICR_BERRCF_S)
#define I2C_ICR_ARLOCF_S                9
#define I2C_ICR_ARLOCF_M                (1 << I2C_ICR_ARLOCF_S)
#define I2C_ICR_OVRCF_S                 10
#define I2C_ICR_OVRCF_M                 (1 << I2C_ICR_OVRCF_S)


#include <stdint.h>

typedef struct
{
        volatile uint32_t CR1;
        volatile uint32_t CR2;
        volatile uint32_t OAR1;
        volatile uint32_t OAR2;
        volatile uint32_t TIMINGR;
        volatile uint32_t reserved0;
        volatile uint32_t ISR;
        volatile uint32_t ICR;
        volatile uint32_t reserved1;
        volatile uint32_t RXDR;
        volatile uint32_t TXDR;
} I2C_TypeDef;

/*
 * Define: I2C_TIMEOUT
 * Количество циклов ожидания установки флага TXIS или RXNE
 *
 */
#define I2C_TIMEOUT_DEFAULT 	1000000		/* Количество циклов ожидания установки флага TXIS или RXNE */
/*
 * Define: I2C_NBYTE_MAX
 * Максимальлное количество байт в посылке (NBYTES)
 *
 */
#define I2C_NBYTE_MAX 	255   /* Максимальное количество байт в посылке (NBYTES) */

#define I2C_ADDRESS_7BIT_MAX 	0x7F   /* Максимальный 7 битный адрес */
#define I2C_ADDRESS_10BIT_MAX 	0x3FF   /* Максимальный 10 битный адрес */

#define I2C_INTMASK 0b11111110 /* Маска для разрешенных прерываний */

/* I2C_error - номера ошибок I2C*/
/* Номер канала */
typedef enum
{
	I2C_ERROR_NONE = 0,    /* Ошибок нет */
	I2C_ERROR_TIMEOUT = 1, /* Превышено ожидание установки флага TXIS или RXNE */
	I2C_ERROR_NACK = 2,    /* Во время передачи не получено подтверждение данных (NACK) */  
	I2C_ERROR_BERR = 4,    /* Ошибка шины */ 
	I2C_ERROR_ARLO = 8,    /* Проигрыш арбитража */
	I2C_ERROR_OVR = 16,     /* Переполнение или недозагрузка */ 
	I2C_ERROR_STOP = 32,   /* Обнаружение STOP на линии */ 
} HAL_I2C_ErrorTypeDef;

/* I2C_addressing_mode - Режим адреса */
typedef enum
{
	I2C_ADDRESSINGMODE_7BIT = 0,    /* 7 битный адрес */
	I2C_ADDRESSINGMODE_10BIT = 1    /* 10 битный адрес */ 
} HAL_I2C_AddressingModeTypeDef;

/* I2C_dual_addressing_mode - Режим дополнительного адреса 7 бит */
typedef enum
{
	I2C_DUALADDRESS_DISABLE = 0,   /* Выключить дополнительный адрес */
	I2C_DUALADDRESS_ENABLE = 1   /* Включить дополнительный адрес */
} HAL_I2C_DualAddressTypeDef;

/* I2C_general_call_mode - Адрес общего вызова */
typedef enum
{
	I2C_GENERALCALL_DISABLE = 0,  	/* Выключить адрес общего вызова */
	I2C_GENERALCALL_ENABLE = 1   	/* Включить адрес общего вызова */
} HAL_I2C_GeneralCallTypeDef;

/* I2C_nostretch_mode - Режим удержания SCL ведомым */
typedef enum
{
	I2C_NOSTRETCH_DISABLE = 0,   /* Растягивание активно */
	I2C_NOSTRETCH_ENABLE = 1   	 /* растягивание выключено */
} HAL_I2C_NoStretchModeTypeDef;

/* I2C_sbc_mode - Режим аппаратного контроля байта ведомым */
typedef enum
{
	I2C_SBC_DISABLE = 0,   /* Аппаратный контроль выключен */
	I2C_SBC_ENABLE = 1     /* Аппаратный контроль включен */
} HAL_I2C_SBCModeTypeDef;

/* I2C_reload_mode - Режим перезаписи NBYTES:  */
typedef enum
{
	I2C_RELOAD_DISABLE = 0,  /* Транзакция завершена после пересылки NBYTES байт данных (на шине ожидаются STOP или RESTART) */
	I2C_RELOAD_ENABLE = 1    /* Транзакция не завершена после пересылки NBYTES байт данных (значение NBYTES будет перезаписано) */
} HAL_I2C_ReloadModeTypeDef;

/* I2C_autoend_mode - Режим автоматического окончания */
typedef enum
{
	I2C_AUTOEND_DISABLE = 0,   /* Режим автоматического окончания отключен */
	I2C_AUTOEND_ENABLE = 1     /* Режим автоматического окончания включен */
} HAL_I2C_AutoEndModeTypeDef;

/* I2C_transfer_direction - Направление передачи */
typedef enum
{
	I2C_TRANSFER_WRITE = 0,   /* Ведущий запрашивает транзакцию записи */
	I2C_TRANSFER_READ = 1    /* Ведущий запрашивает транзакцию чтения */
} HAL_I2C_TransferDirectionTypeDef;

/* I2C_OwnAddress2_mask - Маска второго собственного адреса */
typedef enum
{
	I2C_OWNADDRESS2_MASK_DISABLE = 0,    /* Нет маски */
	I2C_OWNADDRESS2_MASK_111111x = 1,    /* Сравниваются только OA2[7:2] */
	I2C_OWNADDRESS2_MASK_11111xx = 2,    /* Сравниваются только OA2[7:3]; */
	I2C_OWNADDRESS2_MASK_1111xxx = 3,    /* Сравниваются только OA2[7:4]; */
	I2C_OWNADDRESS2_MASK_111xxxx = 4,    /* Сравниваются только OA2[7:5]; */
	I2C_OWNADDRESS2_MASK_11xxxxx = 5,    /* Сравниваются только OA2[7:6]; */
	I2C_OWNADDRESS2_MASK_1xxxxxx = 6,    /* Сравниваются только OA2[7]; */
	I2C_OWNADDRESS2_MASK_1111111 = 7     /* OA2[7:1] маскируются, подтверждаются (ACK) все 7-битные адреса (кроме зарезервированных) */
} HAL_I2C_OwnAddress2MaskTypeDef;

/* I2C_digital_filter - Цифровой фильтр */
typedef enum
{
	I2C_DIGITALFILTER_OFF           = 0,
	I2C_DIGITALFILTER_1CLOCKCYCLES  = 1,
	I2C_DIGITALFILTER_2CLOCKCYCLES  = 2,
	I2C_DIGITALFILTER_15CLOCKCYCLES = 15
} HAL_I2C_DigitalFilterTypeDef;

/* I2C_analog_filter - Цифровой фильтр */
typedef enum
{
	I2C_ANALOGFILTER_ENABLE = 0,
	I2C_ANALOGFILTER_DISABLE = 1
} HAL_I2C_AnalogFilterTypeDef;

typedef enum 
{
  HAL_I2C_MODE_MASTER = 0, /* Режим ведущего */
  HAL_I2C_MODE_SLAVE = 1,  /* Режим ведомого */

} HAL_I2C_ModeTypeDef;

#endif/*__I2C_MIK32_H*/
