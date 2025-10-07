/*
 *  mcu_init.h
 *
 *  Created on: October 6, 2025
 *  Author: Ibrahim Oladepo
 */


#ifndef MCU_INIT_H
#define MCU_INIT_H

#include "stm32f401xe.h"
#include <stdint.h>


void MCU_InitSystemClock(void);
void MCU_Init(void);

#endif