/*
 *  gpio.c
 *
 *  Created on: September 14, 2025
 *  Author: Ibrahim Oladepo
 */

#include "gpio.h"
#include "core_cm4.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 * @brief             - Enables or disables peripheral clock for the given GPIO port
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @return            - None
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi){
    // ENABLE
    if (EnorDi == ENABLE){
        // ENABLE BY GPIO
        if (pGPIOx == GPIOA){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        }
        else if (pGPIOx == GPIOB){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        }
        else if (pGPIOx == GPIOC){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        }
        else if (pGPIOx == GPIOD){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        }
        else if (pGPIOx == GPIOE){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        }
        else if (pGPIOx == GPIOH){
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        }
    }
    else {
        // DISABLE BY GPIO
        if (pGPIOx == GPIOA){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
        }
        else if (pGPIOx == GPIOB){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
        }
        else if (pGPIOx == GPIOC){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
        }
        else if (pGPIOx == GPIOD){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
        }
        else if (pGPIOx == GPIOE){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
        }
        else if (pGPIOx == GPIOH){
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
        }
    }

}


/*********************************************************************
 * @fn      		  - GPIO_ConfigPinMode
 * @brief             - Configures a GPIO pin's mode
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_ConfigPinMode(GPIO_Handler_t *pGPIOHandle){
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
        // Non-interrupt mode
        // CLEAR first, then SET
        pGPIOHandle->pGPIOx->MODER &= ~(0b11 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    }
    else{
        // Configure pin as input
        pGPIOHandle->pGPIOx->MODER &= ~(0b11 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
        pGPIOHandle->pGPIOx->MODER |= (GPIO_MODE_IN << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));

        // Interrupt mode
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
            // Falling trigger
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
            // Rising trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
            // Falling or rising trigger
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // Enable the APB2 Bus clock where is where SYSCFG is connected to
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

        // Configure the GPIO port selection in SYSCFG_EXTICR
        // Selects which GPIO port (A, B, C, etc.) is connected to a specific EXTI line (0-15)
        uint8_t temp_ICRReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp_ICRPortOffset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t temp_ICRPortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        SYSCFG->EXTICR[temp_ICRReg] = temp_ICRPortCode << (temp_ICRPortOffset * 4);

        // Enable the EXTI delivery using the IMR register
        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
}


/*********************************************************************
 * @fn      		  - GPIO_ConfigPinSpeed
 * @brief             - Configures a GPIO pin's speed
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_ConfigPinSpeed(GPIO_Handler_t *pGPIOHandle){
    // CLEAR first, then SET
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0b11 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
}


/*********************************************************************
 * @fn      		  - GPIO_ConfigPinPullMode
 * @brief             - Configures a GPIO pin's pull mode
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_ConfigPinPullMode(GPIO_Handler_t *pGPIOHandle){
    // CLEAR first, then SET
    pGPIOHandle->pGPIOx->PUPDR &= ~(0b11 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
    pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
}


/*********************************************************************
 * @fn      		  - GPIO_ConfigOutputType
 * @brief             - Configures a GPIO pin's output type
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_ConfigOutputType(GPIO_Handler_t *pGPIOHandle){
    // CLEAR first, then SET
    pGPIOHandle->pGPIOx->OTYPER &= ~(0b11 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
}


/*********************************************************************
 * @fn      		  - GPIO_ConfigAlternateFunction
 * @brief             - Configures a GPIO pin's alternate function
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_ConfigAlternateFunction(GPIO_Handler_t *pGPIOHandle){
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8){
        // CLEAR first, then SET for GPIO_AFRL register
        pGPIOHandle->pGPIOx->AFR[0] &= ~(0b1111 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4));
        pGPIOHandle->pGPIOx->AFR[0] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 4));
    }
    else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 16){
        // CLEAR first, then SET for GPIO_AFRH register
        pGPIOHandle->pGPIOx->AFR[1] &= ~(0b1111 << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) * 4));
        pGPIOHandle->pGPIOx->AFR[1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8) * 4));
    }
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 * @brief             - This function initializes a GPIO port
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_Init(GPIO_Handler_t *pGPIOHandle){
    // 1. Enable peripheral clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    // 2. Configure GPIO pin mode
    GPIO_ConfigPinMode(pGPIOHandle);

    // 3. Configure GPIO pin speed
    GPIO_ConfigPinSpeed(pGPIOHandle);

    // 4. Configure GPIO pull mode
    GPIO_ConfigPinPullMode(pGPIOHandle);

    // 5. Configure GPIO output type
    GPIO_ConfigOutputType(pGPIOHandle);

    // 6. Configure GPIO alternate function
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
        GPIO_ConfigAlternateFunction(pGPIOHandle);
    }    
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 * @brief             - disables a GPIO port by reseting the AHB1RSTR register
 * @param[in]         - Pointer to GPIO Handler
 * @return            - None
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx){
    // RESET by GPIO
    // Turn SET first to ENABLE, then RESET to DISABLE
    if (pGPIOx == GPIOA){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOARST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOARST;
    }
    else if (pGPIOx == GPIOB){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOBRST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOBRST;
    }
    else if (pGPIOx == GPIOC){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOCRST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOCRST;
    }
    else if (pGPIOx == GPIOD){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIODRST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIODRST;
    }
    else if (pGPIOx == GPIOE){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOERST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOERST;
    }
    else if (pGPIOx == GPIOH){
        RCC->AHB1RSTR |= RCC_AHB1RSTR_GPIOHRST;
        RCC->AHB1RSTR &= ~RCC_AHB1RSTR_GPIOHRST;
    }
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 * @brief             - reads a value from a given GPIO pin
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number to be read
 * @param[out]        - value read from the pin (0 or 1)
 * @note              - data on IO pins are sampled into the input data register every AHB clock cycle
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber){
    uint8_t pinState;
    pinState = ((pGPIOx->IDR) & (1 << pinNumber));

    return pinState;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 * @brief             - reads values from a given GPIO port
 * @param[in]         - base address of the GPIO peripheral
 * @param[out]        - the value read from the port
 * @note              - bits 31 to 16 of the IDR are reserved; so data present only on first 16 bits
 */
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx){
    uint16_t portState;
    portState = (uint16_t) pGPIOx->IDR;

    return portState;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 * @brief             - writes a value to given GPIO pin
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin number to be written to
 * @param[in]         - value to write to the pin
 * @note              - bits 31 to 16 of the ODR are reserved; so data present only on first 16 bits
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber, uint8_t value){
    if (value == GPIO_PIN_SET){
        pGPIOx->ODR |= (1 << pinNumber);
    }
    else{
        pGPIOx->ODR &= ~(1 << pinNumber);
    }
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 * @brief             - writes a value to a given GPIO port
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - value to write to the port
 */
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t value){
    pGPIOx->ODR = value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 * @brief             - toggles a given GPIO pin
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - pin position to toggle
 */
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t pinNumber){
    pGPIOx->ODR ^= (1 << pinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfigs
 * @brief             - configures GPIO interrupt function and priority on the CPU side
 * @param[in]         - interrupt request number
 * @param[in]         - Enable or disable mode
 */
void GPIO_IRQConfigs(IRQn_Type IRQNumber, uint32_t IRQPriority, uint8_t EnorDi){
    if (EnorDi == ENABLE){
        __NVIC_SetPriority(IRQNumber, IRQPriority);
        __NVIC_EnableIRQ(IRQNumber);
    }
    else{
        __NVIC_DisableIRQ(IRQNumber);
    }
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 * @brief             - clears EXTI PR (pending register) bit to avoid retriggering of interrupt
 * @param[in]         - GPIO pin number
 */
void GPIO_IRQHandling(GPIO_Pin_Number_t pinNumber){
    if (EXTI->PR & (1 << pinNumber)){
        EXTI->PR |= (1 << pinNumber);
    }
}
