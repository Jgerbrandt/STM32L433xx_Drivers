/*
 * stm32l433xx_gpio_driver.c
 *
 *  Created on: May 6, 2025
 *      Author: jessegerbrandt
 */

#include "stm32l433xx_gpio_driver.h"

/*
 * Peripheral Clock Setup
 */

void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable) {
    if (enable == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        }
    }
}


/*
 * Init and De init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    // Configure pin mode
    uint32_t temp = 0;

    // Check if its an interrupt mode
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        // Clear existing type bit
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        // Set new type bit
        pGPIOHandle->pGPIOx->MODER |= temp;
    }

    else {
        // Interrupt
    }

    // Configure Output type
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    // Clear existing speed bits
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    // Set new speed bits
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    // Configure pin speed
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    // Configure pull-up/pull-down
    temp = 0;
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    // Clear existing PUPD bits
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    // Set new PUPD bits
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // Configure Alternate Functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        uint8_t temp1, temp2;
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; // AFR index (0 for pins 0–7, 1 for 8–15)
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; // Bit position within AFRx
        // Clear existing AF bits
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        // Set new AF value
        pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2 );
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    switch ((uint32_t)pGPIOx) {
        case (uint32_t)GPIOA:
            GPIOA_REG_RESET(); break;
        case (uint32_t)GPIOB:
            GPIOB_REG_RESET(); break;
        case (uint32_t)GPIOC:
            GPIOC_REG_RESET(); break;
        case (uint32_t)GPIOD:
            GPIOD_REG_RESET(); break;
        case (uint32_t)GPIOE:
            GPIOE_REG_RESET(); break;
        case (uint32_t)GPIOH:
            GPIOH_REG_RESET(); break;
    }
}

/*
 * Read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001); // Shift down to get a 1 or 0
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t state) {
    if (state == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t state) {
    pGPIOx->ODR = state;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ config and ISR handler
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable) {

}
void GPIO_IRQHandler(uint8_t PinNumber) {

}
