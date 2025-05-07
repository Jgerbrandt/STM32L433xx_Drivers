/*
 * stm32l433xx.h
 *
 *  Created on: May 6, 2025
 *      Author: jessegerbrandt
 */

#ifndef INC_STM32L433XX_H_
#define INC_STM32L433XX_H_

#include <stdint.h>

/*
 * Base addresses of Flash and SRAM memory
 */

#define FLASH_BASE_ADDR 0x08000000UL
#define SRAM1_BASE_ADDR 0x20000000UL
#define SRAM            SRAM1_BASE_ADDR
#define ROM             0x1FFF0000UL

/*
 * AHBx and APBx bus base addresses
 */

#define PERIPH_BASE_ADDR         0x40000000UL
#define APB1_PERIPH_BASE_ADDR    PERIPH_BASE
#define APB2_PERIPH_BASE_ADDR    0x40010000UL
#define AHB1_PERIPH_BASE_ADDR    0x40020000UL
#define AHB2_PERIPH_BASE_ADDR    0x48000000UL

/*
 * Base addresses of peripherals on AHB1 bus
 */

#define RCC_BASE_ADDR       (AHB1_PERIPH_BASE_ADDR + 0x1000UL)

/*
 * Base addresses of GPIO peripherals on AHB2 bus
 */

#define GPIOA_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x0000UL)
#define GPIOB_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x0400UL)
#define GPIOC_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x0800UL)
#define GPIOD_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x0C00UL)
#define GPIOE_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x1000UL)
#define GPIOH_BASE_ADDR     (AHB2_PERIPH_BASE_ADDR + 0x1C00UL)

/*
 * Base addresses of peripherals on APB1 bus
 */

#define I2C1_BASE_ADDR      (APB1_PERIPH_BASE_ADDR + 0x5400UL)


/*
 * Base addresses of peripherals on APB2 bus
 */

#define EXTI_BASE_ADDR      (APB2_PERIPH_BASE_ADDR + 0x0400UL)

typedef struct {
    volatile uint32_t MODER;     // GPIO port mode register
    volatile uint32_t OTYPER;    // GPIO port output type register
    volatile uint32_t OSPEEDR;   // GPIO port output speed register
    volatile uint32_t PUPDR;     // GPIO port pull-up/pull-down register
    volatile uint32_t IDR;       // GPIO port input data register
    volatile uint32_t ODR;       // GPIO port output data register
    volatile uint32_t BSRR;      // GPIO port bit set/reset register
    volatile uint32_t LCKR;      // GPIO port configuration lock register
    volatile uint32_t AFR[2];    // AFR[0] : GPIO alternate function low register, AFR[1] : GPIO alternate function high register
    volatile uint32_t BRR;       // GPIO port bit reset register
}GPIO_RegDef_t;

typedef struct {
    volatile uint32_t CR;             // 0x00: Clock control register
    volatile uint32_t ICSCR;          // 0x04: Internal clock source calibration
    volatile uint32_t CFGR;           // 0x08: Clock configuration register
    volatile uint32_t PLLCFGR;        // 0x0C: PLL configuration register
    volatile uint32_t PLLSAI1CFGR;    // 0x10: PLLSAI1 configuration register
    uint32_t RESERVED0;               // 0x14: Reserved
    volatile uint32_t CIER;           // 0x18: Clock interrupt enable register
    volatile uint32_t CIFR;           // 0x1C: Clock interrupt flag register
    volatile uint32_t CICR;           // 0x20: Clock interrupt clear register
    uint32_t RESERVED1;               // 0x24: Reserved
    volatile uint32_t AHB1RSTR;       // 0x28: AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;       // 0x2C: AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;       // 0x30: AHB3 peripheral reset register
    uint32_t RESERVED2;               // 0x34: Reserved
    volatile uint32_t APB1RSTR1;      // 0x38: APB1 peripheral reset register 1
    volatile uint32_t APB1RSTR2;      // 0x3C: APB1 peripheral reset register 2
    volatile uint32_t APB2RSTR;       // 0x40: APB2 peripheral reset register
    uint32_t RESERVED3;               // 0x44: Reserved
    volatile uint32_t AHB1ENR;        // 0x48: AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;        // 0x4C: AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;        // 0x50: AHB3 peripheral clock enable register
    uint32_t RESERVED4;               // 0x54: Reserved
    volatile uint32_t APB1ENR1;       // 0x58: APB1 peripheral clock enable register 1
    volatile uint32_t APB1ENR2;       // 0x5C: APB1 peripheral clock enable register 2
    volatile uint32_t APB2ENR;        // 0x60: APB2 peripheral clock enable register
    uint32_t RESERVED5;               // 0x64: Reserved
    volatile uint32_t AHB1SMENR;      // 0x68: AHB1 peripheral clocks enable in Sleep and Stop modes
    volatile uint32_t AHB2SMENR;      // 0x6C: AHB2 peripheral clocks enable in Sleep and Stop modes
    volatile uint32_t AHB3SMENR;      // 0x70: AHB3 peripheral clocks enable in Sleep and Stop modes
    uint32_t RESERVED6;               // 0x74: Reserved
    volatile uint32_t APB1SMENR1;     // 0x78: APB1 peripheral clocks enable in Sleep and Stop modes 1
    volatile uint32_t APB1SMENR2;     // 0x7C: APB1 peripheral clocks enable in Sleep and Stop modes 2
    volatile uint32_t APB2SMENR;      // 0x80: APB2 peripheral clocks enable in Sleep and Stop modes
    uint32_t RESERVED7;               // 0x84: Reserved
    volatile uint32_t CCIPR;          // 0x88: Peripherals independent clock configuration register
    uint32_t RESERVED8;               // 0x8C: Reserved
    volatile uint32_t BDCR;           // 0x90: Backup domain control register
    volatile uint32_t CSR;            // 0x94: Control/status register
    volatile uint32_t CRRCR;          // 0x98: Clock recovery RC register
    volatile uint32_t CCIPR2;         // 0x9C: Additional peripheral independent clock configuration
} RCC_RegDef_t;

/*
 * peripheral definitions
 */

#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD       ((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE       ((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASE_ADDR)

#define RCC         ((RCC_RegDef_t*)RCC_BASE_ADDR)

/*
 * Clock enable macros for GPIO peripherals
 */

#define GPIOA_PCLK_EN()     (RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB2ENR |= (1 << 4))
#define GPIOH_PCLK_EN()     (RCC->AHB2ENR |= (1 << 7))

/*
 * Clock disable macros for GPIO peripherals
 */

#define GPIOA_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()     (RCC->AHB2ENR &= ~(1 << 7))

/*
 * Clock enable macros for I2C peripherals
 */

#define I2C_PCLK_EN()       (RCC->APB1ENR1 |= (1 << 21))

/*
 * Clock disable macros for I2C peripherals
 */

#define I2C_PCLK_DI()       (RCC->APB1ENR1 &= ~(1 << 21))

/*
 * Macros to reset GPIO peripherals
 */

#define GPIOA_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOH_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/*
 * Generic Macros
 */

#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET


#endif /* INC_STM32L433XX_H_ */
