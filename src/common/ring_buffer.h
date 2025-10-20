/*
 *  ring_buffer.h
 *
 *  Created on: October 14, 2025
 *  Author: Ibrahim Oladepo
 */


#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>


/*
 *	Circular buffer (FIFO) that can store elements of any size
 */
typedef struct {
    uint8_t *pBuffer;
    uint8_t BufferSize;
    uint8_t HeadIndex;      // Index of new empty slot
    uint8_t TailIndex;      // Index of the oldest element 
    bool Full;              // Condition head == tail applies when full and empty
}Ring_Buffer_t;


void RingBufferPut(Ring_Buffer_t *pRB, uint8_t Data);
uint8_t RingBufferGet(Ring_Buffer_t *pRB);
uint8_t RingBufferPeek(const Ring_Buffer_t *pRB);
bool RingBufferEmpty(const Ring_Buffer_t *pRB);
bool RingBufferFull(const Ring_Buffer_t *pRB);

#endif