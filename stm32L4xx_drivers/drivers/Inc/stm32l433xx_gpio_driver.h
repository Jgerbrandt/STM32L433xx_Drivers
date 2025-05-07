/*
 * stm32l433xx_gpio_driver.h
 *
 *  Created on: May 6, 2025
 *      Author: jessegerbrandt
 */

#ifndef INC_STM32L433XX_GPIO_DRIVER_H_
#define INC_STM32L433XX_GPIO_DRIVER_H_

#include "stm32l433xx.h"

typedef struct {
    uint8_t GPIO_PinNumber;        // GPIO pin number (0–15)
    uint8_t GPIO_PinMode;          // Pin mode: input, output, alternate function, analog, or interrupt
    uint8_t GPIO_PinSpeed;         // Output speed: low, medium, fast, high
    uint8_t GPIO_PinPuPdControl;   // Pull-up/pull-down configuration: none, pull-up, pull-down
    uint8_t GPIO_PinOPType;        // Output type: push-pull or open-drain
    uint8_t GPIO_PinAltFuncMode;   // Alternate function mode (AF0–AF15) if using alternate function
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t *pGPIOx; // base address of GPIO port
    GPIO_PinConfig_t GPIO_PinConfig; // GPIO pin configuration settings
}GPIO_Handle_t;

/*
 * GPIO pin numbers
 */

#define GPIO_PIN_0      0
#define GPIO_PIN_1      1
#define GPIO_PIN_2      2
#define GPIO_PIN_3      3
#define GPIO_PIN_4      4
#define GPIO_PIN_5      5
#define GPIO_PIN_6      6
#define GPIO_PIN_7      7
#define GPIO_PIN_8      8
#define GPIO_PIN_9      9
#define GPIO_PIN_10     10
#define GPIO_PIN_11     11
#define GPIO_PIN_12     12
#define GPIO_PIN_13     13
#define GPIO_PIN_14     14
#define GPIO_PIN_15     15

/*
 * GPIO pin modes
 */

#define GPIO_MODE_IN         0 // input
#define GPIO_MODE_OUT        1 // output
#define GPIO_MODE_ALTFN      2 // alternate function
#define GPIO_MODE_ANALOG     3 // analog
#define GPIO_MODE_IT_FT      4 // interrupt falling trigger
#define GPIO_MODE_IT_RT      5 // interrupt rising trigger
#define GPIO_MODE_IT_RFT     6 // interrupt both edges

/*
 * GPIO pin output types
 */

#define GPIO_OP_TYPE_PP     0 // push-pull
#define GPIO_OP_TYPE_OD     1 // open-drain

/*
 * GPIO output speed
 */

#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3

/*
 * GPIO pin pull up and pull down configuration
 */

#define GPIO_NO_PUPD        0 // No pull-up or pull-down
#define GPIO_PU             1 // pull-up
#define GPIO_PD             2 // pull-down

/*
 * Peripheral Clock Setup
 */

void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enable);

/*
 * Init and De init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t state);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t state);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ config and ISR handler
 */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enable);
void GPIO_IRQHandler(uint8_t PinNumber);



#endif /* INC_STM32L433XX_GPIO_DRIVER_H_ */
