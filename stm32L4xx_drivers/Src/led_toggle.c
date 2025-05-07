/*
 * led_toggle.c
 *
 *  Created on: May 6, 2025
 *      Author: jessegerbrandt
 */

#include "stm32l433xx.h"
#include "stm32l433xx_gpio_driver.h"

void delay(void) {
    for(uint32_t i = 0; i < 500000; i++);
}

int main(void) {
    GPIO_Handle_t gpio_led = {0};

    gpio_led.pGPIOx = GPIOB;
    gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


    GPIO_PClockControl(GPIOB, ENABLE);
    GPIO_Init(&gpio_led);

    while(1) {
        GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        delay();
    }

    return 0;
}


