//
// Created by lei on 2024/4/11.
//

#ifndef UNTITLED_FIFO_H
#define UNTITLED_FIFO_H

#include <stdint.h>
#include "stdbool.h"

#define RX_BUF_SIZE 128

#define FIFO_SIZE 1024// 根据需要调整FIFO大小
typedef struct {
    uint8_t buffer[FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} FIFO_TypeDef;

void FIFO_Init(FIFO_TypeDef *fifo);
bool FIFO_Push(FIFO_TypeDef *fifo, const uint8_t *data, uint16_t length);
bool FIFO_Pop(FIFO_TypeDef *fifo, uint8_t *data, uint16_t length);





#endif