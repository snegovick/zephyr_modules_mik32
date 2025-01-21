#ifndef __SOC_GPIO_H
#define __SOC_GPIO_H

/* registers definitions */
#define MIK32_GPIO_STATE(gpiox)        REG32((gpiox) + 0x00000000U)    /*!< GPIO port state register */
#define MIK32_GPIO_CLEAR(gpiox)        REG32((gpiox) + 0x00000004U)    /*!< GPIO port clear register */
#define MIK32_GPIO_DIROUT(gpiox)       REG32((gpiox) + 0x00000008U)    /*!< GPIO port direction out register */
#define MIK32_GPIO_DIRIN(gpiox)        REG32((gpiox) + 0x0000000CU)    /*!< GPIO port direction in register */
#define MIK32_GPIO_OUTPUT(gpiox)       REG32((gpiox) + 0x00000010U)    /*!< GPIO port output register */
#define MIK32_GPIO_CTL(gpiox)          REG32((gpiox) + 0x00000014U)    /*!< GPIO port control register */

#define MIK32_GPIO_NUMBER(gpiox) (gpiox - GPIO_0_BASE_ADDRESS)/0x400
#define MIK32_PAD_STEP 0xc
#define MIK32_PAD_OFFSET(gpiox, reg_offset) REG32(PAD_CONFIG_BASE_ADDRESS + MIK32_PAD_STEP * MIK32_GPIO_NUMBER(gpiox) + reg_offset)
#define MIK32_PAD_CFG(gpiox) MIK32_PAD_OFFSET(gpiox, 0x0)
#define MIK32_PAD_DS(gpiox) MIK32_PAD_OFFSET(gpiox, 0x4)
#define MIK32_PAD_PUPD(gpiox) MIK32_PAD_OFFSET(gpiox, 0x8)

#define MIK32_GPIO_IRQ_REG_OFFSET(reg_offset)	REG32(GPIO_IRQ_BASE_ADDRESS + reg_offset)
#define MIK32_GPIO_IRQ_STATE			MIK32_GPIO_IRQ_REG_OFFSET(0x0)
#define MIK32_GPIO_IRQ_LINE_MUX			MIK32_GPIO_IRQ_REG_OFFSET(0x4)
#define MIK32_GPIO_IRQ_INTERRUPT		MIK32_GPIO_IRQ_REG_OFFSET(0x8)
#define MIK32_GPIO_IRQ_ENABLE_SET		MIK32_GPIO_IRQ_REG_OFFSET(0xC)
#define MIK32_GPIO_IRQ_ENABLE_CLEAR		MIK32_GPIO_IRQ_REG_OFFSET(0x10)
#define MIK32_GPIO_IRQ_EDGE			MIK32_GPIO_IRQ_REG_OFFSET(0x14)
#define MIK32_GPIO_IRQ_LEVEL			MIK32_GPIO_IRQ_REG_OFFSET(0x18)
#define MIK32_GPIO_IRQ_LEVEL_SET		MIK32_GPIO_IRQ_REG_OFFSET(0x1C)
#define MIK32_GPIO_IRQ_LEVEL_CLEAR		MIK32_GPIO_IRQ_REG_OFFSET(0x20)
#define MIK32_GPIO_IRQ_ANY_EDGE_SET		MIK32_GPIO_IRQ_REG_OFFSET(0x24)
#define MIK32_GPIO_IRQ_ANY_EDGE_CLEAR		MIK32_GPIO_IRQ_REG_OFFSET(0x28)
#define MIK32_GPIO_IRQ_CLEAR			MIK32_GPIO_IRQ_REG_OFFSET(0x2C)

#define MIK32_NUM_IRQ_MUX_LINES 8

#endif/*__SOC_GPIO_H*/
