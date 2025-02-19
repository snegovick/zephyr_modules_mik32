# Модуль поддержки MIK32 Амур для Zephyr RTOS

Пример использования модуля можно посмотреть в репозитории https://github.com/snegovick/zephyr_mik32_example .

# Cтатус

| Модуль | Наличие драйвера | Проверено |
| ------------- | ------------- | ------------- |
| Контроллер прерываний EPIC | :white_check_mark: | :question: |
| Контроллер прерываний GPIO | :white_check_mark: | :x: |
| Контроллер тактирования pmgr | :white_check_mark: | :white_check_mark: |
| USART\* | :white_check_mark: | :white_check_mark: |
| I2C\*\* | :white_check_mark: | :white_check_mark: |
| SPI\*\* | :white_check_mark: | :white_check_mark: |
| EEPROM | :x: | :x: |
| NOR / SPIFI | :x: | :x: |
| GPIO | :white_check_mark: | :white_check_mark: |
| PINCTRL | :white_check_mark: | :question: |
| TIMER16 | :x: | :x: |
| TIMER32 | :x: | :x: |
| MACHINE\_TIMER | :white_check_mark: | :white_check_mark: |
| Crypto | :x: | :x: |
| CRC32 | :x: | :x: |
| OTP | :x: | :x: |
| RTC | :x: | :x: |
| WakeUp | :x: | :x: |
| Watchdog | :x: | :x: |
| VMON | :x: | :x: |
| DMA | :x: | :x: |

\* - Только polling режим
\*\* - Только master, irq

# Замечания

## SPI master

1. При использовании SPI в режиме master, необходимо включить подтяжку на пин NSSIN.
2. При использовании SPI в режиме master CPHA/CPOL mode 3, необходимо включить подтяжку на пин CLK.
