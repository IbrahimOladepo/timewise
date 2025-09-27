/*
 *  gpio.c
 *
 *  Created on: September 14, 2025
 *  Author: Ibrahim Oladepo
 */

#include "stm32f401xe.h"
#include "gpio.h"


void delay(void){
	for (uint32_t i = 0; i < 500000; i++);
}


void GPIO_LEDPinInit(void){
    GPIO_Handler_t GPIOLED;

	GPIOLED.pGPIOx = GPIOA;

	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOLED);
}


void test_GPIO_LEDToggle(void){
	GPIO_LEDPinInit();

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}
}


int main(void){

    TEST();

}

