#ifndef USARTX_H_
#define USARTX_H_

#include "stdint.h"
#include "stm32f4xx_hal.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define SBUS_ALL_CHANNELS      0x00

/*== YS-IRTM 红外模块 (USART4, 仅中断接收, 3字节帧, 无DMA) ======== */

#define IR_TIMEOUT_MS     500

void YS_IRTM_Init(void);
uint8_t YS_IRTM_GetKey(void);
uint8_t YS_IRTM_HasFrame(void);
uint32_t YS_IRTM_GetLastRecvTime(void);

#endif /* USARTX_H_ */