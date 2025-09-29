#include <stdint.h>

// #if !defined(__SOFT_FP__) && defined(__ARM_FP)
//   #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
// #endif

#include "stm32f401xe.h"
#include "gpio.h"


void delay(void){
	for (uint32_t i = 0; i < 500000; i++);
}


int main(void){

	GPIO_Handler_t GPIOLED;

	GPIOLED.pGPIOx = GPIOA;

	GPIOLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIOLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// Enable clock using clock control API
	// GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Handler_t GPIOMCO;
	GPIOMCO.pGPIOx = GPIOC;
	GPIOMCO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIOMCO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOMCO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOMCO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU; // GPIO_NO_PUPD
	GPIOMCO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_0; // GPIO_AF_0

	// Initialize GPIO(s)
	GPIO_Init(&GPIOLED);
	GPIO_Init(&GPIOMCO);

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

	return 0;

}