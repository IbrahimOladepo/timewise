/*
 *  ring_buffer.c
 *
 *  Created on: October 14, 2025
 *  Author: Ibrahim Oladepo
 */

#include "ring_buffer.h"


void RingBufferPut(Ring_Buffer_t *pRB, uint8_t Data){
    pRB->pBuffer[pRB->HeadIndex] = Data;
    pRB->HeadIndex++;

    if (pRB->HeadIndex == pRB->BufferSize){
        pRB->HeadIndex = 0;
    }
}


uint8_t RingBufferGet(Ring_Buffer_t *pRB){
    const uint8_t Data = pRB->pBuffer[pRB->TailIndex];
    pRB->TailIndex++;

    if (pRB->TailIndex == pRB->BufferSize){
        pRB->TailIndex = 0;
    }

    return Data;
}


uint8_t RingBufferPeek(const Ring_Buffer_t *pRB){
    return pRB->pBuffer[pRB->TailIndex];
}


bool RingBufferEmpty(const Ring_Buffer_t *pRB){
    return pRB->HeadIndex == pRB->TailIndex;
}


bool RingBufferFull(const Ring_Buffer_t *pRB){
    uint8_t IndexAfterHead = pRB->HeadIndex + 1;

    if (IndexAfterHead == pRB->BufferSize){
        IndexAfterHead = 0;
    }

    return IndexAfterHead == pRB->TailIndex;
}
