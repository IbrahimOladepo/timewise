/*
 *  i2c.h
 *
 *  Created on: December 26, 2025
 *  Author: Ibrahim Oladepo
 */

#ifndef I2C_H
#define I2C_H

#include "stm32f401xe.h"
#include <stdint.h>

/*
 *  @I2C_CLOCK_SPEEDS
 *	I2C peripheral possible clock speeds
 */
typedef enum {
    I2C_SPEED_100KHZ = 0,
    I2C_SPEED_400KHZ,
    I2C_SPEED_1MHZ
}I2C_Speed_t;

/*
 *  @I2C_DUTY_CYCLE
 *	I2C peripheral duty cycle for fast mode
 */
typedef enum {
    I2C_DUTY_2 = 0,
    I2C_DUTY_16_9
}I2C_Duty_Cycle_t;

/*
 *  @I2C_ADDRESSING_MODE
 *	I2C peripheral addressing modes
 */
typedef enum {
    I2C_ADDR_7BIT = 0,
    I2C_ADDR_10BIT
}I2C_Addressing_Mode_t;

/*
 *  @I2C_ACK_CONTROL
 *	I2C peripheral ACK control
 */
typedef enum {
    I2C_ACK_DISABLE = 0,
    I2C_ACK_ENABLE
}I2C_ACK_Ctrl_t;

/*
 *  @I2C_STRETCH_MODE
 *	I2C peripheral clock stretching mode
 */
typedef enum {
    I2C_STRETCH_ENABLE = 0,
    I2C_STRETCH_DISABLE
}I2C_Stretch_Mode_t;

/*
 *  @I2C_DIRECTION
 *	I2C transfer direction
 */
typedef enum {
    I2C_DIRECTION_WRITE = 0,
    I2C_DIRECTION_READ
}I2C_Direction_t;

/*
 *  @I2C_STATUS
 *	I2C status codes
 */
typedef enum {
    I2C_STATUS_OK = 0,
    I2C_STATUS_ERROR,
    I2C_STATUS_BUSY,
    I2C_STATUS_TIMEOUT,
    I2C_STATUS_NACK
}I2C_Status_t;

/*
 *	Configuration structure for I2C peripheral
 */
typedef struct {
    I2C_Speed_t I2C_Speed;                    // Possible values from @I2C_CLOCK_SPEEDS
    I2C_Duty_Cycle_t I2C_DutyCycle;           // Possible values from @I2C_DUTY_CYCLE
    I2C_Addressing_Mode_t I2C_AddressingMode; // Possible values from @I2C_ADDRESSING_MODE
    I2C_ACK_Ctrl_t I2C_ACKCtrl;               // Possible values from @I2C_ACK_CONTROL
    I2C_Stretch_Mode_t I2C_StretchMode;        // Possible values from @I2C_STRETCH_MODE
    uint32_t I2C_OwnAddress1;                 // Own address 1 for slave mode
    uint8_t I2C_GeneralCall;                  // General call enable/disable
    uint8_t I2C_DualAddress;                  // Dual address mode enable/disable
    uint32_t I2C_OwnAddress2;                 // Own address 2 for dual address mode
}I2C_PinConfig_t;

/*
 *	Handle structure for I2C peripheral
 */
typedef struct {
    // Pointer to hold the base address of the I2C peripheral
    I2C_TypeDef *pI2Cx;                        // Base address of the I2C peripheral
    I2C_PinConfig_t I2C_PinConfig;            // Holds I2C configuration settings
}I2C_Handler_t;

/************************************************************************************
 * 		API supported by this driver
 * 		For more information about the APIs, check the function definitions
 ************************************************************************************/

// Peripheral clock setup
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

// General configs
void I2C_ConfigClockSpeed(I2C_Handler_t *pI2CHandle);
void I2C_ConfigDutyCycle(I2C_Handler_t *pI2CHandle);
void I2C_ConfigAddressingMode(I2C_Handler_t *pI2CHandle);
void I2C_ConfigACK(I2C_Handler_t *pI2CHandle);
void I2C_ConfigStretch(I2C_Handler_t *pI2CHandle);
void I2C_ConfigOwnAddress(I2C_Handler_t *pI2CHandle);
void I2C_ConfigGeneralCall(I2C_Handler_t *pI2CHandle);
void I2C_ConfigDualAddress(I2C_Handler_t *pI2CHandle);

// Init and De-init
void I2C_Init(I2C_Handler_t *pI2CHandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);

// Peripheral control
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

// Data transfer functions (polling mode)
I2C_Status_t I2C_MasterSendData(I2C_TypeDef *pI2Cx, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);
I2C_Status_t I2C_MasterReceiveData(I2C_TypeDef *pI2Cx, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr);
I2C_Status_t I2C_MemRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, uint8_t regAddr, uint8_t RegSize, uint8_t *pRxBuffer, uint8_t BufferSize);
I2C_Status_t I2C_MemRead_16BitMem(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, uint16_t regAddr, uint8_t RegSize, uint8_t *pRxBuffer, uint8_t BufferSize);

// Status flag management
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName);
void I2C_ClearFlag(I2C_TypeDef *pI2Cx, uint32_t FlagName);

// Utility functions
void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx);
void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx);
void I2C_ManageACK(I2C_TypeDef *pI2Cx, uint8_t EnorDi);

#endif /* I2C_H */
