/*
 *  usart.h
 *
 *  Created on: October 12, 2025
 *  Author: Ibrahim Oladepo
 */

#ifndef USART_H
#define USART_H

#include "stm32f401xe.h"
#include <stdint.h>


/*
 *  @USART_OVERSAMPLING_MODES
 *	USART peripheral possible oversampling modes
 */
typedef enum {
    USART_OVER16 = 0,
    USART_OVER8
}USART_Oversampling_Mode_t;


/*
 *  @USART_PARTIY_CONTROL
 *	USART peripheral parity control modes
 */
typedef enum {
    USART_PARITY_DISABLED = 0,
    USART_PARITY_ENABLED
}USART_Parity_Ctrl_t;


/*
 *  @USART_PARTIY_SELECTION
 *	USART peripheral parity selection options
 */
typedef enum {
    USART_PARITY_EVEN = 0,
    USART_PARITY_ODD
}USART_Parity_Selection_t;


/*
 *  @USART_CLOCK_POLARITY
 *	USART peripheral clock polarity
 */
typedef enum {
    USART_CLK_ACTIVE_HIGH = 0,
    USART_CLK_ACTIVE_LOW
}USART_Clk_Polarity_t;


/*
 *  @USART_CLOCK_PHASE
 *	USART peripheral clock phase
 */
typedef enum {
    USART_CLK_CAPON_FIRST = 0,
    USART_CLK_CAPON_SECOND
}USART_Clk_Phase_t;


/*
 *  @USART_BAUDRATE
 *	USART peripheral baudrate options
 */
typedef enum {
    USART_BAUD_9600 = 0,
    USART_BAUD_19200,
    USART_BAUD_115200,
    USART_BAUD_921600
}USART_Baudrate_t;


/*
 *  @USART_TXRX_MODE
 *	USART peripheral Tx Rx mode
 */
typedef enum {
    USART_MODE_RX_ONLY = 0,
    USART_MODE_TX_ONLY,
    USART_MODE_TXRX
}USART_TxRx_Mode_t;


/*
 *	Configuration structure for USART peripheral
 */
typedef struct {
    USART_Oversampling_Mode_t USART_OverMode;       // Possible values from @USART_OVERSAMPLING_MODES
    USART_Parity_Ctrl_t USART_ParityCtrl;           // Possible values from @
    USART_Parity_Selection_t USART_ParitySelect;    // Possible values from @
    USART_Clk_Polarity_t USART_CPOL;                // Possible values from @
    USART_Clk_Phase_t USART_CPHA;                   // Possible values from @
    USART_Baudrate_t USART_Baudrate;                // Possible values from @
    USART_TxRx_Mode_t USART_TxRx_Mode;
}USART_PinConfig_t;

/*
 *	Handle structure for a GPIO pin
 */

typedef struct {
	// Pointer to hold the base address of the GPIO peripheral
	USART_TypeDef *pUSARTx;				// Base address of the GPIO to which the pin belongs
	USART_PinConfig_t USART_PinConfig;	// Holds GPIO pin configuration settings
}USART_Handler_t;


// Peripheral clock setup
void USART_PeriClockControl(USART_TypeDef *pUSARTx, uint8_t EnorDi);

// General configs
void USART_ConfigOversamplingMode(USART_Handler_t *pUSARTHandle);
void USART_ConfigParity(USART_Handler_t *pUSARTHandle);
void USART_ConfigClkPhasePol(USART_Handler_t *pUSARTHandle);
void USART_ConfigBaudRate(USART_Handler_t *pUSARTHandle);
void USART_ConfigTxRx(USART_Handler_t *pUSARTHandle);
void USART_PeriCtrl(USART_TypeDef *pUSARTx, uint8_t EnorDi);

// // Init and De-init
void USART_Init(USART_Handler_t *pUSARTHandle);
void USART_DeInit(USART_TypeDef *pUSARTx);
void USART_TxInterruptInit(USART_TypeDef *pUSARTx);
void USART_TxInterruptDeInit(USART_TypeDef *pUSARTx);

// // Data read and write
void USART_WriteChar_Polling(USART_TypeDef *pUSARTx, char c);
void USART_WriteChar_Interrupt(USART_TypeDef *pUSARTx, char c);
void USART_TxStart(USART_TypeDef *pUSARTx);

// Interrupt
void USART_IRQHandling(USART_TypeDef *pUSARTx);

#endif