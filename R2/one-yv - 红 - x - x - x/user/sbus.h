#ifndef SBUS_H_
#define SBUS_H_


#include "stdint.h"
#include "stm32f4xx_hal.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS      0x00


void SBUS_Reveive(uint8_t data);
void SBUS_Handle(void);
void sbus_loop(void);
void SBUS_Init(void);

uint8_t SBUS_GetStatus(void);

int16_t* SBUS_GetChannels(void);


#endif /* BSP_SBUS_H_ */