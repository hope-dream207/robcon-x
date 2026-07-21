#include <stdint.h>
#include <stdbool.h>
#include "fifo.h"


void FIFO_Init(FIFO_TypeDef *fifo) {
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}
bool FIFO_Push(FIFO_TypeDef *fifo, const uint8_t *data, uint16_t length) {
    if(fifo->count + length > FIFO_SIZE) {
        return false; // 没有足够空间
    }

    for(uint16_t i = 0; i < length; i++) {
        fifo->buffer[fifo->tail] = data[i];
        fifo->tail = (fifo->tail + 1) % FIFO_SIZE;
    }
    fifo->count += length;
    return true;
}
bool FIFO_Pop(FIFO_TypeDef *fifo, uint8_t *data, uint16_t length) {
    if(fifo->count < length) {
        return false; // 请求的数据量大于队列中的元素数量
    }

    for(uint16_t i = 0; i < length; i++) {
        data[i] = fifo->buffer[fifo->head];
        fifo->head = (fifo->head + 1) % FIFO_SIZE;
    }
    fifo->count -= length;
    return true;
}
