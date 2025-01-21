#ifndef __MIK32_IRQ_H
#define __MIK32_IRQ_H

#include <zephyr/soc/mik32_memory_map.h>

#define EPIC_TIMER32_0_INDEX        0
#define EPIC_UART_0_INDEX           1
#define EPIC_UART_1_INDEX           2
#define EPIC_SPI_0_INDEX            3
#define EPIC_SPI_1_INDEX            4
#define EPIC_GPIO_IRQ_INDEX         5
#define EPIC_I2C_0_INDEX            6
#define EPIC_I2C_1_INDEX            7
#define EPIC_WDT_INDEX              8
#define EPIC_TIMER16_0_INDEX        9
#define EPIC_TIMER16_1_INDEX        10
#define EPIC_TIMER16_2_INDEX        11
#define EPIC_TIMER32_1_INDEX        12
#define EPIC_TIMER32_2_INDEX        13
#define EPIC_SPIFI_INDEX            14
#define EPIC_RTC_INDEX              15
#define EPIC_EEPROM_INDEX           16
#define EPIC_WDT_DOM3_INDEX         17
#define EPIC_WDT_SPIFI_INDEX        18
#define EPIC_WDT_EEPROM_INDEX       19
#define EPIC_DMA_INDEX              20
#define EPIC_FREQ_MON_INDEX         21
#define EPIC_PVD_AVCC_UNDER         22
#define EPIC_PVD_AVCC_OVER          23
#define EPIC_PVD_VCC_UNDER          24
#define EPIC_PVD_VCC_OVER           25
#define EPIC_BATTERY_NON_GOOD       26
#define EPIC_BOR_INDEX              27
#define EPIC_TSENS_INDEX            28
#define EPIC_ADC_INDEX              29
#define EPIC_DAC0_INDEX             30
#define EPIC_DAC1_INDEX             31

#ifdef MIK32V0
    /*
    * Defines: Маска линии прерывания 
    *
    * 
    * HAL_EPIC_TIMER32_0_MASK       - Маска для линии прерывания Timer32_0 
    * HAL_EPIC_UART_0_MASK          - Маска для линии прерывания USART_0 
    * HAL_EPIC_UART_1_MASK          - Маска для линии прерывания USART_1 
    * HAL_EPIC_SPI_0_MASK           - Маска для линии прерывания SPI_0 
    * HAL_EPIC_SPI_1_MASK           - Маска для линии прерывания SPI_1 
    * HAL_EPIC_GPIO_IRQ_MASK        - Маска для линии прерывания GPIO 
    * HAL_EPIC_I2C_0_MASK           - Маска для линии прерывания I2C_0 
    * HAL_EPIC_I2C_1_MASK           - Маска для линии прерывания I2C_1 
    * HAL_EPIC_WDT_MASK             - Маска для линии прерывания Сторожевой таймер 
    * HAL_EPIC_TIMER16_0_MASK       - Маска для линии прерывания Timer16_0 
    * HAL_EPIC_TIMER16_1_MASK       - Маска для линии прерывания Timer16_1 
    * HAL_EPIC_TIMER16_2_MASK       - Маска для линии прерывания Timer16_2 
    * HAL_EPIC_TIMER32_1_MASK       - Маска для линии прерывания Timer32_1 
    * HAL_EPIC_TIMER32_2_MASK       - Маска для линии прерывания Timer32_2 
    * HAL_EPIC_SPIFI_MASK           - Маска для линии прерывания SPIFI 
    * HAL_EPIC_RTC_MASK             - Маска для линии прерывания RTC 
    * HAL_EPIC_EEPROM_MASK          - Маска для линии прерывания EEPROM 
    * HAL_EPIC_WDT_DOM3_MASK        - Маска для линии прерывания Сторожевой таймер шины (периферийные устройства) 
    * HAL_EPIC_WDT_SPIFI_MASK       - Маска для линии прерывания Сторожевой таймер шины (SPIFI) 
    * HAL_EPIC_WDT_EEPROM_MASK      - Маска для линии прерывания Сторожевой таймер шины (EEPROM) 
    * HAL_EPIC_DMA_GLB_ERR_MASK     - Маска для линии прерывания ПДП глобальное прерывание 
    * HAL_EPIC_DMA_CHANNELS_MASK    - Маска для линии прерывания ПДП локальное прерывание  
    * HAL_EPIC_FREQ_MON_MASK        - Маска для линии прерывания Монитор частоты  
    * HAL_EPIC_PVD_AVCC_UNDER       - Маска для линии прерывания Монитор напряжения AVCC (ниже порога) 
    * HAL_EPIC_PVD_AVCC_OVER        - Маска для линии прерывания Монитор напряжения AVCC (выше порога)  
    * HAL_EPIC_PVD_VCC_UNDER        - Маска для линии прерывания Монитор напряжения VCC (ниже порога)  
    * HAL_EPIC_PVD_VCC_OVER         - Маска для линии прерывания Монитор напряжения VCC (выше порога)  
    * HAL_EPIC_BATTERY_NON_GOOD     - Маска для линии прерывания Недостаточное напряжение батареи 
    * HAL_EPIC_BOR_MASK             - Маска для линии прерывания BrouwnOut детектор  
    * HAL_EPIC_TSENS_MASK           - Маска для линии прерывания Монитор температуры 
    * HAL_EPIC_ADC_MASK             - Маска для линии прерывания АЦП 
    *
    */
    #define HAL_EPIC_TIMER32_0_MASK         ( 1 << EPIC_TIMER32_0_INDEX )
    #define HAL_EPIC_UART_0_MASK            ( 1 << EPIC_UART_0_INDEX )
    #define HAL_EPIC_UART_1_MASK            ( 1 << EPIC_UART_1_INDEX )
    #define HAL_EPIC_SPI_0_MASK             ( 1 << EPIC_SPI_0_INDEX )
    #define HAL_EPIC_SPI_1_MASK             ( 1 << EPIC_SPI_1_INDEX )
    #define HAL_EPIC_GPIO_IRQ_MASK          ( 1 << EPIC_GPIO_IRQ_INDEX )
    #define HAL_EPIC_I2C_0_MASK             ( 1 << EPIC_I2C_0_INDEX )
    #define HAL_EPIC_I2C_1_MASK             ( 1 << EPIC_I2C_1_INDEX )
    #define HAL_EPIC_WDT_MASK               ( 1 << EPIC_WDT_INDEX )
    #define HAL_EPIC_TIMER16_0_MASK         ( 1 << EPIC_TIMER16_0_INDEX )
    #define HAL_EPIC_TIMER16_1_MASK         ( 1 << EPIC_TIMER16_1_INDEX )
    #define HAL_EPIC_TIMER16_2_MASK         ( 1 << EPIC_TIMER16_2_INDEX )
    #define HAL_EPIC_TIMER32_1_MASK         ( 1 << EPIC_TIMER32_1_INDEX )
    #define HAL_EPIC_TIMER32_2_MASK         ( 1 << EPIC_TIMER32_2_INDEX )
    #define HAL_EPIC_SPIFI_MASK             ( 1 << EPIC_SPIFI_INDEX )
    #define HAL_EPIC_RTC_MASK               ( 1 << EPIC_RTC_INDEX )
    #define HAL_EPIC_EEPROM_MASK            ( 1 << EPIC_EEPROM_INDEX )
    #define HAL_EPIC_WDT_DOM3_MASK          ( 1 << EPIC_WDT_DOM3_INDEX )
    #define HAL_EPIC_WDT_SPIFI_MASK         ( 1 << EPIC_WDT_SPIFI_INDEX )
    #define HAL_EPIC_WDT_EEPROM_MASK        ( 1 << EPIC_WDT_EEPROM_INDEX )
    #define HAL_EPIC_DMA_GLB_ERR_MASK       ( 1 << EPIC_DMA_GLB_ERR_INDEX )
    #define HAL_EPIC_DMA_CHANNELS_MASK      ( 1 << EPIC_DMA_CHANNELS_INDEX )
    #define HAL_EPIC_FREQ_MON_MASK          ( 1 << EPIC_FREQ_MON_INDEX )
    #define HAL_EPIC_PVD_AVCC_UNDER         ( 1 << EPIC_PVD_AVCC_UNDER )
    #define HAL_EPIC_PVD_AVCC_OVER          ( 1 << EPIC_PVD_AVCC_OVER )
    #define HAL_EPIC_PVD_VCC_UNDER          ( 1 << EPIC_PVD_VCC_UNDER )
    #define HAL_EPIC_PVD_VCC_OVER           ( 1 << EPIC_PVD_VCC_OVER )
    #define HAL_EPIC_BATTERY_NON_GOOD       ( 1 << EPIC_BATTERY_NON_GOOD )
    #define HAL_EPIC_BOR_MASK               ( 1 << EPIC_BOR_INDEX )
    #define HAL_EPIC_TSENS_MASK             ( 1 << EPIC_TSENS_INDEX )
    #define HAL_EPIC_ADC_MASK               ( 1 << EPIC_ADC_INDEX )

    /*
    * macros: Проверка флагов линий прерываний в EPIC
    *
    * 
    * EPIC_CHECK_Timer32_0            - Проверка флага Timer32_0
    * EPIC_CHECK_UART_0               - Проверка флага USART_0
    * EPIC_CHECK_UART_1               - Проверка флага USART_1
    * EPIC_CHECK_SPI_0                - Проверка флага SPI_0
    * EPIC_CHECK_SPI_1                - Проверка флага SPI_1
    * EPIC_CHECK_GPIO                 - Проверка флага GPIO
    * EPIC_CHECK_I2C_0                - Проверка флага I2C_0
    * EPIC_CHECK_I2C_1                - Проверка флага I2C_1
    * EPIC_CHECK_WDT                  - Проверка флага Сторожевой таймер
    * EPIC_CHECK_TIMER16_0            - Проверка флага Timer16_0
    * EPIC_CHECK_TIMER16_1            - Проверка флага Timer16_1
    * EPIC_CHECK_TIMER16_2            - Проверка флага Timer16_2
    * EPIC_CHECK_TIMER32_1            - Проверка флага Timer32_1
    * EPIC_CHECK_TIMER32_2            - Проверка флага Timer32_2
    * EPIC_CHECK_SPIFI                - Проверка флага SPIFI
    * EPIC_CHECK_RTC                  - Проверка флага RTC
    * EPIC_CHECK_EEPROM               - Проверка флага EEPROM
    * EPIC_CHECK_WDT_DOM3             - Проверка флага Сторожевой таймер шины (периферийные устройства)
    * EPIC_CHECK_WDT_SPIFI            - Проверка флага Сторожевой таймер шины (SPIFI)
    * EPIC_CHECK_WDT_EEPROM           - Проверка флага Сторожевой таймер шины (EEPROM)
    * EPIC_CHECK_DMA_GLB_ERR          - Проверка флага ПДП глобальное прерывание
    * EPIC_CHECK_DMA_CHANNELS         - Проверка флага ПДП локальное прерывание 
    * EPIC_CHECK_FREQ_MON             - Проверка флага Монитор частоты 
    * EPIC_CHECK_PVD_AVCC_UNDER       - Проверка флага Монитор напряжения AVCC (ниже порога)
    * EPIC_CHECK_PVD_AVCC_OVER        - Проверка флага Монитор напряжения AVCC (выше порога) 
    * EPIC_CHECK_PVD_VCC_UNDER        - Проверка флага Монитор напряжения VCC (ниже порога) 
    * EPIC_CHECK_PVD_VCC_OVER         - Проверка флага Монитор напряжения VCC (выше порога) 
    * EPIC_CHECK_BATTERY_NON_GOOD     - Проверка флага Недостаточное напряжение батареи
    * EPIC_CHECK_BOR                  - Проверка флага BrouwnOut детектор 
    * EPIC_CHECK_TSENS                - Проверка флага Монитор температуры
    * EPIC_CHECK_ADC                  - Проверка флага АЦП
    *
    */
    #define EPIC_CHECK_TIMER32_0()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_0_INDEX))
    #define EPIC_CHECK_UART_0()                 (EPIC->RAW_STATUS & (1 << EPIC_UART_0_INDEX))
    #define EPIC_CHECK_UART_1()                 (EPIC->RAW_STATUS & (1 << EPIC_UART_1_INDEX))
    #define EPIC_CHECK_SPI_0()                  (EPIC->RAW_STATUS & (1 << EPIC_SPI_0_INDEX))
    #define EPIC_CHECK_SPI_1()                  (EPIC->RAW_STATUS & (1 << EPIC_SPI_1_INDEX))
    #define EPIC_CHECK_GPIO_IRQ()               (EPIC->RAW_STATUS & (1 << EPIC_GPIO_IRQ_INDEX))
    #define EPIC_CHECK_I2C_0()                  (EPIC->RAW_STATUS & (1 << EPIC_I2C_0_INDEX))
    #define EPIC_CHECK_I2C_1()                  (EPIC->RAW_STATUS & (1 << EPIC_I2C_1_INDEX))
    #define EPIC_CHECK_WDT()                    (EPIC->STATUS & (1 << EPIC_WDT_INDEX))
    #define EPIC_CHECK_TIMER16_0()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_0_INDEX))
    #define EPIC_CHECK_TIMER16_1()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_1_INDEX))
    #define EPIC_CHECK_TIMER16_2()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_2_INDEX))
    #define EPIC_CHECK_TIMER32_1()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_1_INDEX))
    #define EPIC_CHECK_TIMER32_2()              (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_2_INDEX))
    #define EPIC_CHECK_SPIFI()                  (EPIC->RAW_STATUS & (1 << EPIC_SPIFI_INDEX))
    #define EPIC_CHECK_RTC()                    (EPIC->RAW_STATUS & (1 << EPIC_RTC_INDEX))
    #define EPIC_CHECK_EEPROM()                 (EPIC->RAW_STATUS & (1 << EPIC_EEPROM_INDEX))
    #define EPIC_CHECK_WDT_DOM3()               (EPIC->RAW_STATUS & (1 << EPIC_WDT_DOM3_INDEX))
    #define EPIC_CHECK_WDT_SPIFI()              (EPIC->RAW_STATUS & (1 << EPIC_WDT_SPIFI_INDEX))
    #define EPIC_CHECK_WDT_EEPROM()             (EPIC->RAW_STATUS & (1 << EPIC_WDT_EEPROM_INDEX))
    #define EPIC_CHECK_DMA_GLB_ERR()            (EPIC->RAW_STATUS & (1 << EPIC_DMA_GLB_ERR_INDEX))
    #define EPIC_CHECK_DMA_CHANNELS()           (EPIC->RAW_STATUS & (1 << EPIC_DMA_CHANNELS_INDEX))
    #define EPIC_CHECK_FREQ_MON()               (EPIC->STATUS & (1 << EPIC_FREQ_MON_INDEX))
    #define EPIC_CHECK_PVD_AVCC_UNDER()         (EPIC->STATUS & (1 << EPIC_PVD_AVCC_UNDER))
    #define EPIC_CHECK_PVD_AVCC_OVER()          (EPIC->STATUS & (1 << EPIC_PVD_AVCC_OVER))
    #define EPIC_CHECK_PVD_VCC_UNDER()          (EPIC->STATUS & (1 << EPIC_PVD_VCC_UNDER))
    #define EPIC_CHECK_PVD_VCC_OVER()           (EPIC->STATUS & (1 << EPIC_PVD_VCC_OVER))
    #define EPIC_CHECK_BATTERY_NON_GOOD()       (EPIC->STATUS & (1 << EPIC_BATTERY_NON_GOOD))
    #define EPIC_CHECK_BOR()                    (EPIC->STATUS & (1 << EPIC_BOR_INDEX))
    #define EPIC_CHECK_TSENS()                  (EPIC->RAW_STATUS & (1 << EPIC_TSENS_INDEX))
    #define EPIC_CHECK_ADC()                    (EPIC->STATUS & (1 << EPIC_ADC_INDEX))
#else // MIK32V2
    #define HAL_EPIC_TIMER32_0_MASK         (1 << EPIC_TIMER32_0_INDEX)        
    #define HAL_EPIC_UART_0_MASK            (1 << EPIC_UART_0_INDEX)           
    #define HAL_EPIC_UART_1_MASK            (1 << EPIC_UART_1_INDEX)           
    #define HAL_EPIC_SPI_0_MASK             (1 << EPIC_SPI_0_INDEX)            
    #define HAL_EPIC_SPI_1_MASK             (1 << EPIC_SPI_1_INDEX)            
    #define HAL_EPIC_GPIO_IRQ_MASK          (1 << EPIC_GPIO_IRQ_INDEX)         
    #define HAL_EPIC_I2C_0_MASK             (1 << EPIC_I2C_0_INDEX)            
    #define HAL_EPIC_I2C_1_MASK             (1 << EPIC_I2C_1_INDEX)            
    #define HAL_EPIC_WDT_MASK               (1 << EPIC_WDT_INDEX)              
    #define HAL_EPIC_TIMER16_0_MASK         (1 << EPIC_TIMER16_0_INDEX)        
    #define HAL_EPIC_TIMER16_1_MASK         (1 << EPIC_TIMER16_1_INDEX)        
    #define HAL_EPIC_TIMER16_2_MASK         (1 << EPIC_TIMER16_2_INDEX)        
    #define HAL_EPIC_TIMER32_1_MASK         (1 << EPIC_TIMER32_1_INDEX)        
    #define HAL_EPIC_TIMER32_2_MASK         (1 << EPIC_TIMER32_2_INDEX)        
    #define HAL_EPIC_SPIFI_MASK             (1 << EPIC_SPIFI_INDEX)            
    #define HAL_EPIC_RTC_MASK               (1 << EPIC_RTC_INDEX)              
    #define HAL_EPIC_EEPROM_MASK            (1 << EPIC_EEPROM_INDEX)           
    #define HAL_EPIC_WDT_DOM3_MASK          (1 << EPIC_WDT_DOM3_INDEX)         
    #define HAL_EPIC_WDT_SPIFI_MASK         (1 << EPIC_WDT_SPIFI_INDEX)        
    #define HAL_EPIC_WDT_EEPROM_MASK        (1 << EPIC_WDT_EEPROM_INDEX)       
    #define HAL_EPIC_DMA_MASK               (1 << EPIC_DMA_INDEX)              
    #define HAL_EPIC_FREQ_MON_MASK          (1 << EPIC_FREQ_MON_INDEX)         
    #define HAL_EPIC_PVD_AVCC_MASK          (1 << EPIC_PVD_AVCC_UNDER)         
    #define HAL_EPIC_PVD_AVCCMASK           (1 << EPIC_PVD_AVCC_OVER)          
    #define HAL_EPIC_PVD_VCC_MASK           (1 << EPIC_PVD_VCC_UNDER)          
    #define HAL_EPIC_PVD_VCCMASK            (1 << EPIC_PVD_VCC_OVER)           
    #define HAL_EPIC_BATTERY_NONMASK        (1 << EPIC_BATTERY_NON_GOOD)       
    #define HAL_EPIC_BOR_MASK               (1 << EPIC_BOR_INDEX)              
    #define HAL_EPIC_TSENS_MASK             (1 << EPIC_TSENS_INDEX)            
    #define HAL_EPIC_ADC_MASK               (1 << EPIC_ADC_INDEX)              
    #define HAL_EPIC_DAC0_MASK              (1 << EPIC_DAC0_INDEX)             
    #define HAL_EPIC_DAC1_MASK              (1 << EPIC_DAC1_INDEX)  

    #define EPIC_CHECK_TIMER32_0()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_0_INDEX))                     
    #define EPIC_CHECK_UART_0()                (EPIC->RAW_STATUS & (1 << EPIC_UART_0_INDEX))                
    #define EPIC_CHECK_UART_1()                (EPIC->RAW_STATUS & (1 << EPIC_UART_1_INDEX))                
    #define EPIC_CHECK_SPI_0()                 (EPIC->RAW_STATUS & (1 << EPIC_SPI_0_INDEX))            
    #define EPIC_CHECK_SPI_1()                 (EPIC->RAW_STATUS & (1 << EPIC_SPI_1_INDEX))            
    #define EPIC_CHECK_GPIO_IRQ()              (EPIC->RAW_STATUS & (1 << EPIC_GPIO_IRQ_INDEX))                
    #define EPIC_CHECK_I2C_0()                 (EPIC->RAW_STATUS & (1 << EPIC_I2C_0_INDEX))            
    #define EPIC_CHECK_I2C_1()                 (EPIC->RAW_STATUS & (1 << EPIC_I2C_1_INDEX))            
    #define EPIC_CHECK_WDT()                   (EPIC->STATUS & (1 << EPIC_WDT_INDEX))            
    #define EPIC_CHECK_TIMER16_0()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_0_INDEX))                
    #define EPIC_CHECK_TIMER16_1()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_1_INDEX))                
    #define EPIC_CHECK_TIMER16_2()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER16_2_INDEX))                
    #define EPIC_CHECK_TIMER32_1()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_1_INDEX))                
    #define EPIC_CHECK_TIMER32_2()             (EPIC->RAW_STATUS & (1 << EPIC_TIMER32_2_INDEX))                
    #define EPIC_CHECK_SPIFI()                 (EPIC->RAW_STATUS & (1 << EPIC_SPIFI_INDEX))            
    #define EPIC_CHECK_RTC()                   (EPIC->RAW_STATUS & (1 << EPIC_RTC_INDEX))            
    #define EPIC_CHECK_EEPROM()                (EPIC->RAW_STATUS & (1 << EPIC_EEPROM_INDEX))                
    #define EPIC_CHECK_WDT_DOM3()              (EPIC->RAW_STATUS & (1 << EPIC_WDT_DOM3_INDEX))                
    #define EPIC_CHECK_WDT_SPIFI()             (EPIC->RAW_STATUS & (1 << EPIC_WDT_SPIFI_INDEX))                
    #define EPIC_CHECK_WDT_EEPROM()            (EPIC->RAW_STATUS & (1 << EPIC_WDT_EEPROM_INDEX))                    
    #define EPIC_CHECK_DMA()                   (EPIC->RAW_STATUS & (1 << EPIC_DMA_INDEX))            
    #define EPIC_CHECK_FREQ_MON()              (EPIC->STATUS & (1 << EPIC_FREQ_MON_INDEX))                
    #define EPIC_CHECK_PVD_AVCC_UNDER()        (EPIC->STATUS & (1 << EPIC_PVD_AVCC_UNDER))                
    #define EPIC_CHECK_PVD_AVCC_OVER()         (EPIC->STATUS & (1 << EPIC_PVD_AVCC_OVER))                
    #define EPIC_CHECK_PVD_VCC_UNDER()         (EPIC->STATUS & (1 << EPIC_PVD_VCC_UNDER))                
    #define EPIC_CHECK_PVD_VCC_OVER()          (EPIC->STATUS & (1 << EPIC_PVD_VCC_OVER))                
    #define EPIC_CHECK_BATTERY_NON_GOOD()      (EPIC->STATUS & (1 << EPIC_BATTERY_NON_GOOD))                    
    #define EPIC_CHECK_BOR()                   (EPIC->STATUS & (1 << EPIC_BOR_INDEX))            
    #define EPIC_CHECK_TSENS()                 (EPIC->RAW_STATUS & (1 << EPIC_TSENS_INDEX))            
    #define EPIC_CHECK_ADC()                   (EPIC->STATUS & (1 << EPIC_ADC_INDEX))            
    #define EPIC_CHECK_DAC0()                  (EPIC->STATUS & (1 << EPIC_DAC0_INDEX))            
    #define EPIC_CHECK_DAC1()                  (EPIC->STATUS & (1 << EPIC_DAC1_INDEX))                       
#endif // MIK32V0

#define HAL_IRQ_EnableInterrupts() do {		\
		set_csr(mstatus, MSTATUS_MIE);	\
		set_csr(mie, MIE_MEIE);		\
	} while(0)

#define HAL_IRQ_DisableInterrupts() do {		\
		clear_csr(mie, MIE_MEIE);		\
	} while(0)

#define HAL_EPIC_MaskEdgeSet(InterruptMask) do {	\
		EPIC->MASK_EDGE_SET |= InterruptMask;	\
	} while(0)

#define HAL_EPIC_MaskEdgeClear(InterruptMask) do {	\
		EPIC->MASK_EDGE_CLEAR |= InterruptMask;	\
	} while(0)

#define HAL_EPIC_MaskLevelSet(InterruptMask) do {	\
		EPIC->MASK_LEVEL_SET = InterruptMask;	\
	} while(0)

#define HAL_EPIC_MaskLevelClear(InterruptMask) do {	\
		EPIC->MASK_LEVEL_CLEAR = InterruptMask;	\
	} while(0)

#define HAL_EPIC_GetStatus() EPIC->STATUS

#define HAL_EPIC_GetRawStatus() EPIC->RAW_STATUS

#endif/* __MIK32_IRQ_H*/
