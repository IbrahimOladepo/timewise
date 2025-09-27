/*
 *  gpio.h
 *
 *  Created on: September 14, 2025
 *  Author: Ibrahim Oladepo
 */

#ifndef GPIO_H
#define GPIO_H

#include "stm32f401xe.h"
#include <stdint.h>


/*
 *  @GPIO_PIN_NUMBERS
 *	GPIO pin possible numbers (macros)
 */
typedef enum {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15
} GPIO_Pin_Number_t;


/*
 *  @GPIO_ALT_FUNCTIONS
 *	GPIO pin possible numbers (macros)
 */
typedef enum {
    GPIO_AF_0 = 0,
    GPIO_AF_1,
    GPIO_AF_2,
    GPIO_AF_3,
    GPIO_AF_4,
    GPIO_AF_5,
    GPIO_AF_6,
    GPIO_AF_7,
    GPIO_AF_8,
    GPIO_AF_9,
    GPIO_AF_10,
    GPIO_AF_11,
    GPIO_AF_12,
    GPIO_AF_13,
    GPIO_AF_14,
    GPIO_AF_15
} GPIO_Alt_Function_t;


/*
 *  @GPIO_PIN_MODES
 *	GPIO pin possible modes (macros)
 */
typedef enum {
    GPIO_MODE_IN = 0,
    GPIO_MODE_OUT,
    GPIO_MODE_ALTFN,
    GPIO_MODE_ANALOG,
    GPIO_MODE_IT_FT,
    GPIO_MODE_IT_RT,
    GPIO_MODE_IT_RFT
} GPIO_Mode_t;


/*
 *  @GPIO_OUTPUT_TYPES
 *	GPIO pin possible output types (macros)
 */
typedef enum {
    GPIO_OP_TYPE_PP = 0,
    GPIO_OP_TYPE_OD
}GPIO_Output_Type_t;
 

/*
 *	@GPIO_PIN_SPEEDS
 *	GPIO pin possible output speeds (macros)
 */
typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_FAST,
    GPIO_SPEED_HIGH,
}GPIO_Pin_Speed_t;


/*
 *	@GPIO_PULL_CONFIG
 *	GPIO pin pull up pull down configuration (macros)
 */
typedef enum {
    GPIO_NO_PUPD = 0,
    GPIO_PU,
    GPIO_PD
}GPIO_Pull_Config_t;


/*
 *	Configuration structure for a GPIO pin
 */
typedef struct {
 	GPIO_Pin_Number_t GPIO_PinNumber;			// Possible values from @GPIO_PIN_NUMBERS
 	GPIO_Mode_t GPIO_PinMode;				    // Possible values from @GPIO_PIN_MODES
 	GPIO_Pin_Speed_t GPIO_PinSpeed;				// Possible values from @GPIO_PIN_SPEEDS
 	GPIO_Pull_Config_t GPIO_PinPuPdControl;		// Possible values from @GPIO_PULL_CONFIG
 	GPIO_Output_Type_t GPIO_PinOPType;			// Possible values from @GPIO_OUTPUT_TYPES
 	GPIO_Alt_Function_t GPIO_PinAltFunMode;     // Possible values from @GPIO_ALT_FUNCTIONS
}GPIO_PinConfig_t;


/*
 *	Handle structure for a GPIO pin
 */

typedef struct {
	// Pointer to hold the base address of the GPIO peripheral
	GPIO_TypeDef *pGPIOx;				// Base address of the GPIO to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;	// Holds GPIO pin configuration settings
}GPIO_Handler_t;


/************************************************************************************
 * 		API supported by this driver
 * 		For more information about the APIs, check the function definitions
 ************************************************************************************/

// Peripheral clock setup
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi);

// General configs
void GPIO_ConfigPinMode(GPIO_Handler_t *pGPIOHandle);
void GPIO_ConfigPinSpeed(GPIO_Handler_t *pGPIOHandle);
void GPIO_ConfigPinPullMode(GPIO_Handler_t *pGPIOHandle);
void GPIO_ConfigOutputType(GPIO_Handler_t *pGPIOHandle);
void GPIO_ConfigAlternateFunction(GPIO_Handler_t *pGPIOHandle);

// Init and De-init
void GPIO_Init(GPIO_Handler_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber);

// // IRQ configuration and ISR handling
// void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
// void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
// void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* GPIO_H */