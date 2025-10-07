/*
 *  gpio.c
 *
 *  Created on: September 14, 2025
 *  Author: Ibrahim Oladepo
 */

#include "stm32f401xe.h"
#include "mcu_init.h"
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


void GPIO_BtnPinInit(void){
    GPIO_Handler_t GPIOBtn;

    GPIOBtn.pGPIOx = GPIOC;

    GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIOBtn);
}


void GPIO_MCO1Init(void){
    // MCU pin connected to PLL
    GPIO_Handler_t GPIOMCO;

	GPIOMCO.pGPIOx = GPIOA;
    
	GPIOMCO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIOMCO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOMCO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOMCO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU; // GPIO_NO_PUPD
	GPIOMCO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0; // GPIO_AF_0

	// Initialize GPIO(s)
	GPIO_Init(&GPIOMCO);
}


void GPIO_MCO2Init(void){
    // MCU pin connected to HSI
    GPIO_Handler_t GPIOMCO;

	GPIOMCO.pGPIOx = GPIOC;
    
	GPIOMCO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIOMCO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOMCO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOMCO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU; // GPIO_NO_PUPD
	GPIOMCO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0; // GPIO_AF_0

	// Initialize GPIO(s)
	GPIO_Init(&GPIOMCO);
}


void test_GPIO_LEDToggle(void){
	GPIO_LEDPinInit();

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}
}


void test_GPIO_LEDOnOff(void){
    GPIO_LEDPinInit();

    while(1){
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay();
    }
}


void test_GPIO_Interrupt(void){
    GPIO_LEDPinInit();
    GPIO_BtnPinInit();

    // Configure interrupt on button pin
    GPIO_IRQConfigs(EXTI15_10_IRQn, 0, ENABLE);

    while (1){
        // Do nothing; wait for interrupt
    }
    
}


void test_System_Clock_Config(void){
    MCU_Init();

    GPIO_MCO1Init();
    GPIO_MCO2Init();

    test_GPIO_LEDOnOff();
}


int main(void){

    TEST();

    return 0;

}


void EXTI15_10_IRQHandler(void){
    // TODO: Add some delay to avoid debounce
    delay();

    GPIO_IRQHandling(GPIO_PIN_13);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}

