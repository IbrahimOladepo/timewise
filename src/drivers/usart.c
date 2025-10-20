/*
 *  uart.c
 *
 *  Created on: October 12, 2025
 *  Author: Ibrahim Oladepo
 */

#include "usart.h"
#include "ring_buffer.h"


#define USART_BUFFER_SIZE   16
static uint8_t Buffer[USART_BUFFER_SIZE];
static Ring_Buffer_t TxBuffer = {.pBuffer = Buffer, .BufferSize = USART_BUFFER_SIZE};


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 * @brief             - Enables or disables clock for a USART peropheral
 * @param[in]         - Pointer to USARTx peripheral base address
 * @param[in]         - Desired instruction: ENABLE or DISABLE 
 * @note              - USART 1 and USART6 on APB2 bus
 *                      USART 2 on APB1 bus
 */
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi){
    if (EnorDi == ENABLE){
        if (pUSARTx == USART1){
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        }
        else if (pUSARTx == USART2){
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        }
        else if (pUSARTx == USART6){
            RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        }
    }
    else{
        if (pUSARTx == USART1){
            RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);
        }
        else if (pUSARTx == USART2){
            RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN);
        }
        else if (pUSARTx == USART6){
            RCC->APB2ENR &= ~(RCC_APB2ENR_USART6EN);
        }
    }
}


/*********************************************************************
 * @fn      		  - USART_ConfigOversamplingMode
 * @brief             - Configures the oversampling mode for a USART peripheral
 * @param[in]         - Pointer to USARTx handler
 * @note              - Options: oversampling by 8 or 16 bits
 */
void USART_ConfigOversamplingMode(USART_Handler_t *pUSARTHandle){
    if (pUSARTHandle->USART_PinConfig.USART_OverMode == USART_OVER8){
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_OVER8;
    }
    else{
        pUSARTHandle->pUSARTx->CR1 &= ~(USART_CR1_OVER8);   // USART_OVER16
    }
}


/*********************************************************************
 * @fn      		  - USART_ConfigParity
 * @brief             - Configures the clock polarity for a USART peripheral
 * @param[in]         - Pointer to USARTx handler
 * @note              - Options: clock active when low or when high
 */
void USART_ConfigParity(USART_Handler_t *pUSARTHandle){
    if (pUSARTHandle->USART_PinConfig.USART_ParityCtrl == USART_PARITY_ENABLED){
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_PCE;

        if (pUSARTHandle->USART_PinConfig.USART_ParitySelect == USART_PARITY_ODD){
            pUSARTHandle->pUSARTx->CR1 |= USART_CR1_PS;
        }
        else{
            pUSARTHandle->pUSARTx->CR1 &= ~(USART_CR1_PS);
        }
    }
    else{   
        pUSARTHandle->pUSARTx->CR1 &= ~(USART_CR1_PCE);    // USART_PARITY_DISABLED
    }
}


/*********************************************************************
 * @fn      		  - USART_ConfigClkPhasePol
 * @brief             - Configures the clock phase and polarity for a USART peripheral
 * @param[in]         - Pointer to USARTx handler
 * @note              - Polarity options: clock active when low or when high
 *                    - Phase options: first or second
 */
void USART_ConfigClkPhasePol(USART_Handler_t *pUSARTHandle){
    if (pUSARTHandle->USART_PinConfig.USART_CPOL == USART_CLK_ACTIVE_LOW){
        pUSARTHandle->pUSARTx->CR2 |= USART_CR2_CPOL;
    }
    else{   
        pUSARTHandle->pUSARTx->CR2 &= ~(USART_CR2_CPOL);    // USART_CLK_ACTIVE_HIGH
    }

    if (pUSARTHandle->USART_PinConfig.USART_CPHA == USART_CLK_CAPON_SECOND){
        pUSARTHandle->pUSARTx->CR2 |= USART_CR2_CPHA;
    }
    else{   
        pUSARTHandle->pUSARTx->CR2 &= ~(USART_CR2_CPHA);    // USART_CLK_CAPON_FIRST
    }
}


/*********************************************************************
 * @fn      		  - USART_ConfigBaudRate
 * @brief             - Configures the baudrate for a USART peripheral
 * @param[in]         - Pointer to USARTx handler
 * @note              - Baudrate options (Bps): 9600, 19200, 115200, and 921600
 *                    - Assumptions: system clock = 84MHz, peripehral clock 1 and 2 = 42MHz
 */
void USART_ConfigBaudRate(USART_Handler_t *pUSARTHandle){
    if (pUSARTHandle->USART_PinConfig.USART_OverMode == USART_OVER16){
        if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_9600){
            // 273.4375 | Mantissa = 273 [0x111], Fraction = 16 × 0.4375 = 7 [0x7]
            pUSARTHandle->pUSARTx->BRR |= (0x111 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0x7 << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_19200){
            // 136.75 | Mantissa = 136 [0x88], Fraction = 16 × 0.75 = 12 [0xC]
            pUSARTHandle->pUSARTx->BRR |= (0x88 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0xC << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_115200){
            // 22.8125 | Mantissa = 22 [0x16], Fraction = 16 × 0.8125 = 13 [0xD]
            pUSARTHandle->pUSARTx->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0xD << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_921600){
            // 2.875 | Mantissa = 2 [0x2], Fraction = 16 × 0.875 = 14 [0xE]
            pUSARTHandle->pUSARTx->BRR |= (0x2 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0xE << USART_BRR_DIV_Fraction_Pos);
        }
    }
    else if (pUSARTHandle->USART_PinConfig.USART_OverMode == USART_OVER8){
        if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_9600){
            // 546.875 | Mantissa = 546 [0x222], Fraction = 16 × 0.875 = 14 [0xE]
            pUSARTHandle->pUSARTx->BRR |= (0x222 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0x8 << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_19200){
            // 273.5 | Mantissa = 273 [0x111], Fraction = 16 × 0.5 = 8 [0x8]
            pUSARTHandle->pUSARTx->BRR |= (0x111 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0x8 << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_115200){
            // 45.625 | Mantissa = 45 [0x2D], Fraction = 16 × 0.625 = 10 [0xA]
            pUSARTHandle->pUSARTx->BRR |= (0x2D << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0xA << USART_BRR_DIV_Fraction_Pos);
        }
        else if (pUSARTHandle->USART_PinConfig.USART_Baudrate == USART_BAUD_921600){
            // 5.75 | Mantissa = 5 [0x5], Fraction = 16 × 0.75 = 12 [0xC]
            pUSARTHandle->pUSARTx->BRR |= (0x5 << USART_BRR_DIV_Mantissa_Pos);
            pUSARTHandle->pUSARTx->BRR |= (0xC << USART_BRR_DIV_Fraction_Pos);
        }
    }
}


/*********************************************************************
 * @fn      		  - USART_PeriCtrl
 * @brief             - Enables or disables USARTx peripheral
 * @param[in]         - Pointer to USARTx peripheral base address
 * @param[in]         - Desired instruction: ENABLE or DISABLE 
 * @note              - USARTx peripheral will not function until enabled
 */
void USART_PeriCtrl(USART_TypeDef *pUSARTx, uint8_t EnorDi){
    if (EnorDi == ENABLE){
        pUSARTx->CR1 |= USART_CR1_UE;
    }
    else{
        pUSARTx->CR1 &= ~(USART_CR1_UE);
    }
}


/*********************************************************************
 * @fn      		  - USART_ConfigTxRx
 * @brief             - Configures USARTx peripheral Tx and Rx mode
 * @param[in]         - Pointer to USARTx handler
 * @note              - ##
 */
void USART_ConfigTxRx(USART_Handler_t *pUSARTHandle){
    if (pUSARTHandle->USART_PinConfig.USART_TxRx_Mode == USART_MODE_RX_ONLY){
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RE;
    }
    else if (pUSARTHandle->USART_PinConfig.USART_TxRx_Mode == USART_MODE_TX_ONLY){
        pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TE;
    }
    else if (pUSARTHandle->USART_PinConfig.USART_TxRx_Mode == USART_MODE_TXRX){
        pUSARTHandle->pUSARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE);
    }
}


/*********************************************************************
 * @fn      		  - USART_Init
 * @brief             - Initializes a USART peripheral
 * @param[in]         - Pointer to USARTx handler
 * @note              - ##
 */
void USART_Init(USART_Handler_t *pUSARTHandle){
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
    USART_ConfigOversamplingMode(pUSARTHandle);
    USART_ConfigParity(pUSARTHandle);
    USART_ConfigClkPhasePol(pUSARTHandle);
    USART_ConfigBaudRate(pUSARTHandle);
    USART_ConfigTxRx(pUSARTHandle);

    // Clear TC register which is 1 on boot
    // pUSARTHandle->pUSARTx->SR &= ~(USART_SR_TC);
}


/*********************************************************************
 * @fn      		  - USART_DeInit
 * @brief             - De-initializes a USART peripheral
 * @param[in]         - Pointer to USARTx peripheral base address
 * @note              - ##
 */
void USART_DeInit(USART_TypeDef *pUSARTx){
    USART_PeriCtrl(pUSARTx, DISABLE);
    USART_PeriClockControl(pUSARTx, DISABLE);
}


/*********************************************************************
 * @fn      		  - USART_WriteChar_Polling
 * @brief             - Writes a character to the USARTx peripheral Tx buffer in a polling way
 * @param[in]         - Pointer to USARTx peripheral base address
 * @param[in]         - Character to write
 * @note              - ##
 */
void USART_WriteChar_Polling(USART_TypeDef *pUSARTx, char c){
    // Wait for any ongoing Tx to complete
    // Wait while the TC flag is NOT set (i.e., wait while TC == 0)
    while ((pUSARTx->SR & USART_SR_TC) == 0);

    pUSARTx->DR |= c;

    // Some terminals expect carriage return (\r) after line-feed (\n) for proper new line
    if (c == '\n'){
        USART_WriteChar_Polling(pUSARTx, '\r');
    }
}


/*********************************************************************
 * @fn      		  - USART_WriteChar_Interrupt
 * @brief             - Writes a character to the USARTx peripheral Tx buffer using interrupt
 * @param[in]         - Pointer to USARTx peripheral base address
 * @param[in]         - Character to write
 * @note              - ##
 */
void USART_WriteChar_Interrupt(USART_TypeDef *pUSARTx, char c){
    // Some terminals expect carriage return (\r) 'BEFORE' line-feed (\n) for proper new line
    if (c == '\n'){
        USART_WriteChar_Interrupt(pUSARTx, '\r');
    }

    // Ensure interrupt cannot occur while the following code is running
    // By disabling and then enabling the interupt after
    // USART_TxInterruptDeInit(pUSARTx);

    // Check for ongoing transmission
    const bool TxOngoing = !RingBufferEmpty(&TxBuffer);
    
    RingBufferPut(&TxBuffer, c);

    if (!TxOngoing){
        // USART_TxStart(pUSARTx);
        // A USART interrupt is generated whenever TXE=1 in the USART_SR register
        pUSARTx->CR1 |= USART_CR1_TXEIE;
    }

    // USART_TxInterruptInit(pUSARTx);
}


/*********************************************************************
 * @fn      		  - USART_TxStart
 * @brief             - Initializes USART Tx data transmission from ring buffer
 */
void USART_TxStart(USART_TypeDef *pUSARTx){
    if (!RingBufferEmpty(&TxBuffer)){
        uint8_t c = RingBufferPeek(&TxBuffer);
        pUSARTx->DR = c;
    }
}


/*********************************************************************
 * @fn      		  - USART_InterruptInit
 * @brief             - Initializes Tx interrupt for a USART peripheral
 * @param[in]         - Pointer to USARTx peripheral base address
 */
void USART_TxInterruptInit(USART_TypeDef *pUSARTx){
    // A USART interrupt is generated whenever TC=1 in the USART_SR register
    pUSARTx->CR1 |= USART_CR1_TCIE;

    // A USART interrupt is generated whenever TXE=1 in the USART_SR register
    pUSARTx->CR1 |= USART_CR1_TXEIE;

    // TODO IN APP: Need to enable IRQ and set priority
}


/*********************************************************************
 * @fn      		  - USART_TxInterruptDeInit
 * @brief             - De-initializes Tx interrupt for a USART peripheral
 * @param[in]         - Pointer to USARTx peripheral base address
 */
void USART_TxInterruptDeInit(USART_TypeDef *pUSARTx){
    pUSARTx->CR1 &= ~(USART_CR1_TCIE);
    pUSARTx->CR1 &= ~(USART_CR1_TXEIE);
}


/*********************************************************************
 * @fn      		  - USART_IRQHandling
 * @brief             - Handles USARTx interrupt requests
 * @param[in]         - Pointer to USARTx peripheral base address
 * @note              - ##
 */
void USART_IRQHandling(USART_TypeDef *pUSARTx){
    // TC induced interrupt handling
    if ((pUSARTx->SR & USART_SR_TC) && (pUSARTx->CR1 & USART_CR1_TCIE)){
        pUSARTx->SR &= ~(USART_SR_TC);
    }
}


void USART1_IRQHandler(void){
    // Check if the interrupt was triggered by TXE (Transmit Data Register Empty)
    if (USART1->SR & USART_SR_TXE){
        // if (RingBufferEmpty(&TxBuffer)){
        //     while(1);   // TODO
        // }

        // // Remove the transmitted data byte from the buffer
        // RingBufferGet(&TxBuffer);

        // // Clear the interrupt to avoid accidental re-triggering
        // USART_IRQHandling(USART1);

        // if (!RingBufferEmpty(&TxBuffer)){
        //     USART_TxStart(USART1);
        // }
        
        uint8_t data = RingBufferGet(&TxBuffer);

        if (data){
            // Data available; buffer not empty
            USART1->DR = data;
        }
        else{
            // Buffer is empty. Disable interrupt for now
            USART1->CR1 &= ~(USART_CR1_TXEIE);
        }
    }
}

