#ifndef SBUS_H_
#define SBUS_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define SBUS_SIGNAL_OK        0x00
#define SBUS_SIGNAL_LOST      0x01
#define SBUS_SIGNAL_FAILSAFE  0x03
#define SBUS_ALL_CHANNELS     0x00

void SBUS_Reveive(uint8_t data);
void SBUS_Handle(void);
void sbus_loop(void);
void SBUS_Start_DMA(void);
uint8_t SBUS_Send_DMA(const uint8_t *data, uint16_t len);

extern int16_t g_sbus_channels[18];

//********** 红外发射（YS-IRTM，USART2）   ********//
#define YS_FA       0xFA
#define YS_F1       0xF1
#define YS_one      0x01
#define YS_two      0x02

void YS_IRTM_Send(uint8_t key);

#endif