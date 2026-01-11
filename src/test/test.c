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
#include "i2c.h"


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


void GPIO_I2C1_Init(void){
    // Initialize GPIO for I2C1 (PB6: SCL, PB7: SDA)
    GPIO_Handler_t GPIOI2C;
    GPIOI2C.pGPIOx = GPIOB;
    GPIOI2C.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    GPIOI2C.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD; // GPIO_OP_TYPE_PP; // Open-drain for I2C
    GPIOI2C.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;   // Pull-up for I2C
    GPIOI2C.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

    // GPIOB->ODR |= (1 << 8) | (1 << 9);  // Set PB8 and PB9 high

    // I2C1 SCL (PB6)
    GPIOI2C.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_4;      // AF4 for I2C1 and I2C2
    GPIOI2C.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;        // GPIO_PIN_6 8 10
    GPIO_Init(&GPIOI2C);

    // I2C1 SDA (PB7)
    GPIOI2C.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_AF_4;      // AF4 for I2C1 | AF9 for I2C2
    GPIOI2C.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;         // GPIO_PIN_7 9 3
    GPIO_Init(&GPIOI2C);
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


void I2C1_Init(void){
    // Initialize I2C1 peripheral
    I2C_Handler_t I2COne;
    I2COne.pI2Cx = I2C1;
    I2COne.I2C_PinConfig.I2C_Speed = I2C_SPEED_100KHZ;
    I2COne.I2C_PinConfig.I2C_DutyCycle = I2C_DUTY_2;
    I2COne.I2C_PinConfig.I2C_AddressingMode = I2C_ADDR_7BIT;
    I2COne.I2C_PinConfig.I2C_ACKCtrl = I2C_ACK_ENABLE;
    I2COne.I2C_PinConfig.I2C_StretchMode = I2C_STRETCH_ENABLE;
    I2COne.I2C_PinConfig.I2C_OwnAddress1 = 0;       // Example slave address
    I2COne.I2C_PinConfig.I2C_GeneralCall = 0;       // Disable general call
    I2COne.I2C_PinConfig.I2C_DualAddress = 0;       // Disable dual address
    I2COne.I2C_PinConfig.I2C_OwnAddress2 = 0;       // Not used when dual address disabled
    
    I2C_Init(&I2COne);
    
    // Enable I2C1 peripheral
    I2C_PeripheralControl(I2C1, ENABLE);
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

    uint32_t temp = (uint32_t) USART1->SR;
    // USART1->DR |= 'X';
    // USART1->SR &= ~(USART_SR_TC);
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
        delay();
        delay();
        delay();
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


void test_USART_Tx_Interrupt(void){
    MCU_Init();
    GPIO_MCO1Init();
    GPIO_MCO2Init();
    GPIO_LEDPinInit();
    USART1_Init();

    // Configure interrupt on USART1
    GPIO_IRQConfigs(USART1_IRQn, 0, ENABLE);

    char counter = 0;
    

    while (1){
        USART_WriteChar_Interrupt(USART1, 'I');
        USART_WriteChar_Interrupt(USART1, 'B');
        USART_WriteChar_Interrupt(USART1, 'T');
        USART_WriteChar_Interrupt(USART1, 'E');
        USART_WriteChar_Interrupt(USART1, 'C');
        USART_WriteChar_Interrupt(USART1, 'H');
        USART_WriteChar_Interrupt(USART1, 'N');
        USART_WriteChar_Interrupt(USART1, 'O');
        USART_WriteChar_Interrupt(USART1, 'L');
        USART_WriteChar_Interrupt(USART1, 'O');
        USART_WriteChar_Interrupt(USART1, 'G');
        USART_WriteChar_Interrupt(USART1, 'Y');
        USART_WriteChar_Interrupt(USART1, ' ');
        USART_WriteChar_Interrupt(USART1, counter);
        USART_WriteChar_Interrupt(USART1, '\n');
        delay();
        delay();
        delay();
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        delay();

        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        delay();
        delay();
        delay();

        counter++;

        if (counter >= 255){
            counter = 0;
        }
    }
}


void test_I2C_Basic(void){
    MCU_Init();
    GPIO_LEDPinInit();
    GPIO_I2C1_Init();
    I2C1_Init();
    
    // uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    // uint8_t rx_buffer[4];

    // IMU: MPU6050 addresses
    uint8_t slave_addr_mpu = 0x68;      // Example EEPROM address | 0x50
    uint8_t reg_addr_mpu = 0x75;

    // VL6160 addresses
    uint8_t slave_addr_vl = 0x29;
    uint16_t reg_addr_vl = 0x0000;     // Identification Model ID

    uint8_t read_data = 0;

    uint8_t counter = 0;
    
    while(1){
        // Test I2C write
        // I2C_Status_t status = I2C_MasterSendData(I2C1, test_data, 4, slave_addr);

        // Read data from slave
        // I2C_Status_t status = I2C_MasterReceiveData(I2C1, read_data, 1, slave_addr);

        // I2C_Status_t status = I2C_MemRead(I2C2, slave_addr_vl, reg_addr_vl, 1, read_data, 1);
        
        I2C_Status_t status = I2C_MemRead_16BitMem(I2C1, slave_addr_vl, reg_addr_vl, 1, &read_data, 1);

        if(status == I2C_STATUS_OK){
            // Success - blink LED fast
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            delay();
        } else {
            // Error - blink LED slow
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            delay();
            delay();
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            delay();
            delay();
            delay();
        }
        
        // delay();
        // delay();

        read_data = 0;
    }
}


void test_I2C_ReadWrite(void){
    MCU_Init();
    GPIO_LEDPinInit();
    GPIO_I2C1_Init();
    I2C1_Init();
    
    uint8_t write_data[] = {0x00, 0x55}; // Address 0x00, data 0x55
    uint8_t read_data[2];
    uint8_t slave_addr = 0x50; // Example EEPROM address
    
    while(1){
        // Write data to slave
        I2C_Status_t write_status = I2C_MasterSendData(I2C1, write_data, 2, slave_addr);
        
        delay(); // Small delay between operations
        
        // Read data from slave
        I2C_Status_t read_status = I2C_MasterReceiveData(I2C1, read_data, 1, slave_addr);
        
        if(write_status == I2C_STATUS_OK && read_status == I2C_STATUS_OK){
            // Success - double blink
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            delay();
        } else {
            // Error - long blink
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            delay();
            delay();
            delay();
            delay();
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            delay();
        }
        
        delay();
        delay();
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
