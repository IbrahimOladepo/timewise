/*
 *  mcu_init.c
 *
 *  Created on: October 6, 2025
 *  Author: Ibrahim Oladepo
 */

#include "mcu_init.h"


/*********************************************************************
 * @fn      		  - MCU_InitClocks
 * @brief             - initialize MCU system clock to 84MHz
 * @note              - PLLN Range: 192 <= PLLN <= 432
 *                    - PLLM Range: 2   <= PLLM <= 63
 *                    - PLLP Options: 2, 4, 6, and 8
 *                    - Recommended to have VCO input as 2MHz to reduce jitter
 *                    - VCO input = HSI / PLLM; thus PLLM has to be 8
 */
void MCU_InitSystemClock(void){
    // 1a. Enable and Wait for HSI to be ready (It should be on by default, but verify)
    RCC->CR |= RCC_CR_HSION;

    // 1b. Verify that HSI is ready
    while ((RCC->CR & RCC_CR_HSIRDY) == RESET);
    // if ((RCC->CR && RCC_CR_HSIRDY) == SET){
        
    // 2a. Turn off the PLL and PLLI2S before configutation
    RCC->CR &= ~(RCC_CR_PLLON);
    // RCC->CR &= ~(RCC_CR_PLLI2SON);

    // 2b. Wait for PLL to be OFF
    while ((RCC->CR & RCC_CR_PLLRDY) != 0); 

    // 3. Configure FLASH access
    // Increase Latency to 2 wait states for SYSCLK up to 90MHz
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

    // 3. Select HSI as the PLL clock source
    // RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);  // DEFAULT

    // 4. Configure PLLP, PLLN, and PLLM for 84MHz system clock
    //    CLEAR first, then SET
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
    RCC->PLLCFGR |= (0b10 << RCC_PLLCFGR_PLLP_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
    RCC->PLLCFGR |= (252 << RCC_PLLCFGR_PLLN_Pos);

    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
    RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);

    // 5. Configure Bus Prescalers
    // AHB Prescaler: No division (HPRE = 0)
    RCC->CFGR &= ~(RCC_CFGR_HPRE);

    // APB1 Prescaler: /2 (PCLK1 max 42MHz)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    
    // APB2 Prescaler: /2 (PCLK2 max 84MHz)
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    // EXTRA: Configure MCO1 to output PLL clock
    RCC->CFGR &= ~(RCC_CFGR_MCO1);      // Clear MCO1 source bits
    // RCC->CFGR |= (0b11 << RCC_CFGR_MCO1_Pos);
    RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);   // Clear MCO1 prescaler bits
    RCC->CFGR |= (0b110 << RCC_CFGR_MCO1PRE_Pos);   // HSI / 4

    RCC->CFGR &= ~(RCC_CFGR_MCO2);      // Clear MCO1 source bits
    RCC->CFGR |= (0b11 << RCC_CFGR_MCO2_Pos);
    RCC->CFGR &= ~(RCC_CFGR_MCO2PRE);   // Clear MCO1 prescaler bits
    RCC->CFGR |= (0b111 << RCC_CFGR_MCO2PRE_Pos);   // SYSCLK / 5

    // 6a. Turn on the PLL and PLLI2S
    RCC->CR |= RCC_CR_PLLON;
    // RCC->CR |= RCC_CR_PLLI2SON;

    // 6b. Wait fot PLL to be ready before setting as system clock source
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // 7. Switch system clock to PLL
    RCC->CFGR &= ~(RCC_CFGR_SW); 
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // 8. Wait for PLL to be used as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0);
    // }   
}


/*********************************************************************
 * @fn      		  - MCU_Init
 * @brief             - initializes general mirocontroller state
 *                      this is mainly for system clock configurations
 * @note              - ###
 */
void MCU_Init(void){
    MCU_InitSystemClock();
}

