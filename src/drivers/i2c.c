/*
 *  i2c.c
 *
 *  Created on: December 26, 2025
 *  Author: Ibrahim Oladepo
 */

#include "i2c.h"

// Private function prototypes
static void I2C_ExecuteAddressPhase(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, I2C_Direction_t Direction);
static I2C_Status_t I2C_WaitForFlag(I2C_TypeDef *pI2Cx, uint32_t FlagName, uint8_t Status, uint32_t Timeout);

/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE){
        if(pI2Cx == I2C1){
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        }
        else if(pI2Cx == I2C2){
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        }
        else if(pI2Cx == I2C3){
            RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        }
    }
    else{
        if(pI2Cx == I2C1){
            RCC->APB1ENR &= ~(RCC_APB1ENR_I2C1EN);
        }
        else if(pI2Cx == I2C2){
            RCC->APB1ENR &= ~(RCC_APB1ENR_I2C2EN);
        }
        else if(pI2Cx == I2C3){
            RCC->APB1ENR &= ~(RCC_APB1ENR_I2C3EN);
        }
    }
}

/*
 * Clock speed configuration
 */
void I2C_ConfigClockSpeed(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CR2;

    // Configure APB1 clock frequency
    tempreg &= ~(0x3F << 0);            // Clear FREQ[5:0]
    tempreg |= 42;                      // APB1 clock is set to 42 MHz
    pI2CHandle->pI2Cx->CR2 = tempreg;

    // Configure clock control register
    tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CCR;

    // Clear CCR bits
    tempreg &= ~(0xFFF << 0);   // Clear CCR[11:0]
    tempreg &= ~(1 << 14);      // Clear DUTY
    tempreg &= ~(1 << 15);      // Clear F/S

    if(pI2CHandle->I2C_PinConfig.I2C_Speed == I2C_SPEED_100KHZ){
        // Standard mode
        // CCR calculation:
        // CCR = (T_I2C / 2) / T_PCLK = F_PCLK / (2 * F_I2C)
        // tempreg |= 0xD2;        // DIRECT CALC FOR 100kHz AT 42MHz PCLK
        tempreg |= (42000000 / (2 * 100000)) & 0xFFF;
        tempreg &= ~(1 << 15);  // F/S = 0 for standard mode
    }
    else if(pI2CHandle->I2C_PinConfig.I2C_Speed == I2C_SPEED_400KHZ){
        // Fast mode
        tempreg |= (1 << 15);   // F/S = 1 for fast mode
        
        if(pI2CHandle->I2C_PinConfig.I2C_DutyCycle == I2C_DUTY_2){
            tempreg |= (42000000 / (3 * 400000)) & 0xFFF; // DUTY = 0
        }
        else{
            tempreg |= (42000000 / (25 * 400000)) & 0xFFF; // DUTY = 1
            tempreg |= (1 << 14); // DUTY = 1
        }
    }

    pI2CHandle->pI2Cx->CCR = tempreg;

    // Configure rise time register
    tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->TRISE;
    
    if(pI2CHandle->I2C_PinConfig.I2C_Speed == I2C_SPEED_100KHZ){
        tempreg = (42000000 / 1000000) + 1; // Standard mode
    }
    else{
        tempreg = (42000000 / 3000000) + 1; // Fast mode
    }

    pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;
}

/*
 * Duty cycle configuration
 */
void I2C_ConfigDutyCycle(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CCR;

    tempreg &= ~(I2C_CCR_DUTY); // Clear DUTY bit

    if(pI2CHandle->I2C_PinConfig.I2C_DutyCycle == I2C_DUTY_16_9){
        tempreg |= I2C_CCR_DUTY;
    }

    pI2CHandle->pI2Cx->CCR = tempreg;
}

/*
 * Addressing mode configuration
 */
void I2C_ConfigAddressingMode(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->OAR1;

    tempreg &= ~(I2C_OAR1_ADDMODE); // Clear ADDMODE bit

    if(pI2CHandle->I2C_PinConfig.I2C_AddressingMode == I2C_ADDR_10BIT){
        tempreg |= I2C_OAR1_ADDMODE;
    }

    pI2CHandle->pI2Cx->OAR1 = tempreg;
}

/*
 * ACK control configuration
 */
void I2C_ConfigACK(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CR1;

    tempreg &= ~(I2C_CR1_ACK); // Clear ACK bit

    if(pI2CHandle->I2C_PinConfig.I2C_ACKCtrl == I2C_ACK_ENABLE){
        tempreg |= I2C_CR1_ACK;
    }

    pI2CHandle->pI2Cx->CR1 = tempreg;
}

/*
 * Clock stretching configuration
 */
void I2C_ConfigStretch(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CR1;

    tempreg &= ~(I2C_CR1_NOSTRETCH); // Clear NOSTRETCH bit

    if(pI2CHandle->I2C_PinConfig.I2C_StretchMode == I2C_STRETCH_DISABLE){
        tempreg |= I2C_CR1_NOSTRETCH;
    }

    pI2CHandle->pI2Cx->CR1 = tempreg;
}

/*
 * Own address configuration
 */
void I2C_ConfigOwnAddress(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->OAR1;

    tempreg &= ~(0x3FF << 0); // Clear ADD[9:0]
    tempreg &= ~(I2C_OAR1_ADDMODE);    // Clear ADDMODE

    tempreg |= (pI2CHandle->I2C_PinConfig.I2C_OwnAddress1 & 0x3FF);

    // Always set bit 14 as required by STM32
    tempreg |= (1 << 14);

    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // Configure second own address if dual address mode is enabled
    if(pI2CHandle->I2C_PinConfig.I2C_DualAddress){
        tempreg = 0;
        tempreg = pI2CHandle->pI2Cx->OAR2;
        tempreg &= ~(0x7F << 1); // Clear ADD2[7:1]
        tempreg &= ~(1 << 0);    // Clear ENDUAL
        
        tempreg |= ((pI2CHandle->I2C_PinConfig.I2C_OwnAddress2 & 0x7F) << 1);
        tempreg |= (1 << 0); // Enable dual address mode

        pI2CHandle->pI2Cx->OAR2 = tempreg;
    }
}

/*
 * General call configuration
 */
void I2C_ConfigGeneralCall(I2C_Handler_t *pI2CHandle)
{
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->CR1;

    tempreg &= ~(I2C_CR1_ENGC); // Clear ENGC bit

    if(pI2CHandle->I2C_PinConfig.I2C_GeneralCall)
    {
        tempreg |= I2C_CR1_ENGC;
    }

    pI2CHandle->pI2Cx->CR1 = tempreg;
}

/*
 * Dual address configuration
 */
void I2C_ConfigDualAddress(I2C_Handler_t *pI2CHandle){
    uint32_t tempreg = 0;
    tempreg = pI2CHandle->pI2Cx->OAR2;

    tempreg &= ~(I2C_OAR2_ENDUAL); // Clear ENDUAL bit

    if(pI2CHandle->I2C_PinConfig.I2C_DualAddress){
        tempreg |= I2C_OAR2_ENDUAL;
    }

    pI2CHandle->pI2Cx->OAR2 = tempreg;
}

/*
 * I2C initialization
 */
void I2C_Init(I2C_Handler_t *pI2CHandle)
{
    // Enable peripheral clock
    I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

    // Configure clock speed
    I2C_ConfigClockSpeed(pI2CHandle);

    // Configure duty cycle (for fast mode)
    I2C_ConfigDutyCycle(pI2CHandle);

    // Configure addressing mode
    I2C_ConfigAddressingMode(pI2CHandle);

    // Configure ACK control
    I2C_ConfigACK(pI2CHandle);

    // Configure clock stretching
    I2C_ConfigStretch(pI2CHandle);

    // Configure own address
    I2C_ConfigOwnAddress(pI2CHandle);

    // Configure general call
    I2C_ConfigGeneralCall(pI2CHandle);

    // Configure dual address
    I2C_ConfigDualAddress(pI2CHandle);
}

/*
 * I2C de-initialization
 */
void I2C_DeInit(I2C_TypeDef *pI2Cx){
    if(pI2Cx == I2C1){
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; 
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);
    }
    else if(pI2Cx == I2C2){
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C2RST);
    }
    else if(pI2Cx == I2C3){
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C3RST);
    }
}

/*
 * Peripheral control
 */
void I2C_PeripheralControl(I2C_TypeDef *pI2Cx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        pI2Cx->CR1 |= I2C_CR1_PE; // Set PE bit
    }
    else{
        pI2Cx->CR1 &= ~(I2C_CR1_PE); // Clear PE bit
    }
}

/*
 * Generate start condition
 */
void I2C_GenerateStartCondition(I2C_TypeDef *pI2Cx){
    pI2Cx->CR1 |= I2C_CR1_START; // Set START bit
}

/*
 * Generate stop condition
 */
void I2C_GenerateStopCondition(I2C_TypeDef *pI2Cx){
    pI2Cx->CR1 |= I2C_CR1_STOP; // Set STOP bit
}

/*
 * Manage ACK
 */
void I2C_ManageACK(I2C_TypeDef *pI2Cx, uint8_t EnorDi){
    if(EnorDi == ENABLE){
        pI2Cx->CR1 |= I2C_CR1_ACK; // Set ACK bit
    }
    else{
        pI2Cx->CR1 &= ~(I2C_CR1_ACK); // Clear ACK bit
    }
}

/*
 * Get flag status
 */
uint8_t I2C_GetFlagStatus(I2C_TypeDef *pI2Cx, uint32_t FlagName){
    if(pI2Cx->SR1 & FlagName){
        return SET;
    }
    
    return RESET;
}

/*
 * Clear flag
 */
void I2C_ClearFlag(I2C_TypeDef *pI2Cx, uint32_t FlagName){
    // Some flags are cleared by writing to SR1
    pI2Cx->SR1 &= ~FlagName;
}

/*
 * Wait for flag with timeout
 */
static I2C_Status_t I2C_WaitForFlag(I2C_TypeDef *pI2Cx, uint32_t FlagName, uint8_t Status, uint32_t Timeout){
    uint32_t tickstart = 0;
    
    // Simple timeout implementation (assuming 1ms per loop iteration)
    while(((pI2Cx->SR1 & FlagName) != (Status ? FlagName : 0)) && (Timeout > 0)){
        Timeout--;
    }

    if(Timeout == 0){
        return I2C_STATUS_TIMEOUT;
    }

    return I2C_STATUS_OK;
}

/*
 * Execute address phase
 */
static void I2C_ExecuteAddressPhase(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, I2C_Direction_t Direction){
    uint32_t tempreg = 0;
    tempreg = (SlaveAddr << 1);

    if(Direction == I2C_DIRECTION_READ){
        tempreg |= (1 << 0); // Set read bit
    }

    pI2Cx->DR = tempreg;
}

/*
 * Master send data (polling mode)
 */
I2C_Status_t I2C_MasterSendData(I2C_TypeDef *pI2Cx, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){
    I2C_Status_t status = I2C_STATUS_OK;
    uint32_t timeout = 10000000; // Timeout value

    // Generate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send slave address with write bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_WRITE);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Clear ADDR flag (by reading SR1 and SR2)
    uint32_t temp = pI2Cx->SR1;         // Possible SR1 is already read in the STATUS check above
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Send data
    for(uint32_t i = 0; i < Len; i++){
        // Wait for TXE flag
        status = I2C_WaitForFlag(pI2Cx, I2C_SR1_TXE, SET, timeout);
        if(status != I2C_STATUS_OK){
            return status;
        }

        pI2Cx->DR = pTxBuffer[i];

        // Generate stop condition after last byte
        // if(i == (Len - 1)){
        //     I2C_GenerateStopCondition(pI2Cx);
        // }
    }

    // Wait for BTF flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_BTF, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Generate stop condition
    // Datasheet: "Stop condition should be programmed during EV8_2 event, when either TxE or BTF is set."
    I2C_GenerateStopCondition(pI2Cx);

    return I2C_STATUS_OK;
}

/*
 * Master receive data (polling mode)
 */
I2C_Status_t I2C_MasterReceiveData(I2C_TypeDef *pI2Cx, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr){
    I2C_Status_t status = I2C_STATUS_OK;
    uint32_t timeout = 10000000; // Timeout value

    // Generate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send slave address with read bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_READ);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);

    // Disable ACK before clearing ADDR
    if(Len == 1){
        I2C_ManageACK(pI2Cx, DISABLE);
    }

    if(status != I2C_STATUS_OK){
        return status;
    }

    // Clear ADDR flag (by reading SR1 and SR2)
    uint32_t temp = pI2Cx->SR1;
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Handle different receive scenarios
    if(Len == 1){
        // Disable ACK before clearing ADDR
        // I2C_ManageACK(pI2Cx, DISABLE);

        // Generate stop condition
        I2C_GenerateStopCondition(pI2Cx);

        // Wait for RXNE flag
        status = I2C_WaitForFlag(pI2Cx, I2C_SR1_RXNE, SET, timeout);
        if(status != I2C_STATUS_OK){
            return status;
        }

        // Read data
        pRxBuffer[0] = pI2Cx->DR;

        // Re-enable ACK
        I2C_ManageACK(pI2Cx, ENABLE);
    }
    else{
        // Enable ACK
        I2C_ManageACK(pI2Cx, ENABLE);

        // Receive all bytes except the last one
        for(uint32_t i = 0; i < Len - 1; i++){
            // Wait for RXNE flag
            status = I2C_WaitForFlag(pI2Cx, I2C_SR1_RXNE, SET, timeout);
            if(status != I2C_STATUS_OK){
                return status;
            }

            pRxBuffer[i] = pI2Cx->DR;
        }

        // Disable ACK before receiving last byte
        I2C_ManageACK(pI2Cx, DISABLE);

        // Generate stop condition
        I2C_GenerateStopCondition(pI2Cx);

        // Wait for RXNE flag for last byte
        status = I2C_WaitForFlag(pI2Cx, I2C_SR1_RXNE, SET, timeout);
        if(status != I2C_STATUS_OK){
            return status;
        }

        pRxBuffer[Len - 1] = pI2Cx->DR;

        // Re-enable ACK
        I2C_ManageACK(pI2Cx, ENABLE);
    }

    return I2C_STATUS_OK;
}

/*
 * Master get memory data (polling mode)
 */
I2C_Status_t I2C_MemRead(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, uint8_t regAddr, uint8_t RegSize, uint8_t *pRxBuffer, uint8_t BufferSize){
    I2C_Status_t status = I2C_STATUS_OK;
    uint32_t timeout = 10000000; // Timeout value

    // Generate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send slave address with write bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_WRITE);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Clear ADDR flag (by reading SR1 and SR2)
    uint32_t temp = pI2Cx->SR1;         // Possible SR1 is already read in the STATUS check above
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Wait for TXE flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_TXE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send register address to read from 
    pI2Cx->DR = regAddr;

    // Regenerate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send slave address with read bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_READ);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Disable ACK before clearing ADDR
    I2C_ManageACK(pI2Cx, DISABLE);

    // Clear ADDR flag (by reading SR1 and SR2)
    temp = pI2Cx->SR1;
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Program STOP immediately after clearing ADDR
    // This tells the hardware: "After the byte currently being shifted in, stop."
    // Datasheet: "Stop condition should be programmed during EV8_2 event, when either TxE or BTF is set."
    I2C_GenerateStopCondition(pI2Cx);

    // Wait for RXNE flag (Receive buffer Not Empty)
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_RXNE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }    

    // Read data
    *pRxBuffer = pI2Cx->DR;     // Should clear RXNE
    
    // Re-enable ACK
    I2C_ManageACK(pI2Cx, ENABLE);

    return I2C_STATUS_OK;
}

/*
 * Master get memory data | 16 bit slave addresses (polling mode)
 */
I2C_Status_t I2C_MemRead_16BitMem(I2C_TypeDef *pI2Cx, uint8_t SlaveAddr, uint16_t regAddr, uint8_t RegSize, uint8_t *pRxBuffer, uint8_t BufferSize){
    I2C_Status_t status = I2C_STATUS_OK;
    uint32_t timeout = 10000000; // Timeout value

    // Generate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send slave address with write bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_WRITE);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Clear ADDR flag (by reading SR1 and SR2)
    uint32_t temp = pI2Cx->SR1;         // Possible SR1 is already read in the STATUS check above
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Wait for TXE flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_TXE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send the first 8 bits (MSB) of the register address to read from 
    pI2Cx->DR = (regAddr >> 8);

    // Wait for TXE flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_TXE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Send the last 8 bits (LSB) of the register address to read from 
    pI2Cx->DR = (regAddr & 0xFF);

    // Wait for TXE flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_TXE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    I2C_GenerateStopCondition(pI2Cx);

    // Regenerate start condition
    I2C_GenerateStartCondition(pI2Cx);

    // Wait for SB flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_SB, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // --- START OF READ PHASE ---
    
    // Send slave address with read bit
    I2C_ExecuteAddressPhase(pI2Cx, SlaveAddr, I2C_DIRECTION_READ);

    // Wait for ADDR flag
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_ADDR, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }

    // Disable ACK before clearing ADDR
    I2C_ManageACK(pI2Cx, DISABLE);

    // Clear ADDR flag (by reading SR1 and SR2)
    temp = pI2Cx->SR1;
    temp = pI2Cx->SR2;
    (void)temp; // Suppress unused variable warning

    // Program STOP immediately after clearing ADDR
    // This tells the hardware: "After the byte currently being shifted in, stop."
    // Datasheet: "Stop condition should be programmed during EV8_2 event, when either TxE or BTF is set."
    I2C_GenerateStopCondition(pI2Cx);

    // Wait for RXNE flag (Receive buffer Not Empty)
    status = I2C_WaitForFlag(pI2Cx, I2C_SR1_RXNE, SET, timeout);
    if(status != I2C_STATUS_OK){
        return status;
    }    

    // Read data
    *pRxBuffer = pI2Cx->DR;     // Should clear RXNE
    
    // Re-enable ACK
    I2C_ManageACK(pI2Cx, ENABLE);

    return I2C_STATUS_OK;
}
