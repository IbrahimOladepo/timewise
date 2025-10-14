/*
 *  gpio.c
 *
 *  Created on: September 14, 2025
 *  Author: Ibrahim Oladepo
 */

#include "stm32f401xe.h"
#include "mcu_init.h"
#include "gpio.h"
#include "usart.h"


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


void USART1_Init(void){
    // Initialize GPIO
    GPIO_Handler_t GPIOUSART;
	GPIOUSART.pGPIOx = GPIOB;
	GPIOUSART.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOUSART.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOUSART.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIOUSART.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_7;

	// USART Tx
    GPIOUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&GPIOUSART);

    // USART Rx
    GPIOUSART.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&GPIOUSART);

    // Initialize USART1 Peripheral
    USART_Handler_t USARTOne;
    USARTOne.pUSARTx = USART1;
    USARTOne.USART_PinConfig.USART_OverMode = USART_OVER16;
    USARTOne.USART_PinConfig.USART_CPHA = USART_CLK_CAPON_SECOND;
    USARTOne.USART_PinConfig.USART_CPOL = USART_CLK_ACTIVE_LOW;
    USARTOne.USART_PinConfig.USART_ParityCtrl = USART_PARITY_DISABLED;
    USARTOne.USART_PinConfig.USART_Baudrate = USART_BAUD_115200;
    USARTOne.USART_PinConfig.USART_TxRx_Mode = USART_MODE_TXRX;
    USART_Init(&USARTOne);

    // Activate USART1 peripheral
    USART_PeriCtrl(USART1, ENABLE);
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


void test_USART_Tx(void){
    MCU_Init();
    GPIO_MCO1Init();
    GPIO_MCO2Init();
    GPIO_LEDPinInit();
    USART1_Init();
    

    while (1){
        USART_WriteChar_Polling(USART1, 'i');
        USART_WriteChar_Polling(USART1, 'b');
        USART_WriteChar_Polling(USART1, 't');
        USART_WriteChar_Polling(USART1, 'e');
        USART_WriteChar_Polling(USART1, 'c');
        USART_WriteChar_Polling(USART1, 'h');
        USART_WriteChar_Polling(USART1, '\n');
        delay();
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay();
    }
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

