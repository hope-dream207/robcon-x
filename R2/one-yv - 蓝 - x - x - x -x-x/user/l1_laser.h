/**
  * @file    l1_laser.h
  * @brief   L1s-40 激光测距模组驱动 (HEX 协议)
  */

#ifndef __L1_LASER_H
#define __L1_LASER_H

#include <stdint.h>
#include "main.h"
#include "modbus.h"

/* ==========================================================================
 * 错误码
 * ========================================================================== */

typedef enum {
    L1_OK          = 0,
    L1_ERR_TIMEOUT = 1,
    L1_ERR_CRC     = 2,
    L1_ERR_BUSY    = 3,
    L1_ERR_HAL     = 4,
} L1_Error_t;

/* ==========================================================================
 * API
 * ========================================================================== */

void         L1_Init(void);
L1_Error_t   L1_ReadDistance(float *distance_m);
void         L1_Restart(void);
const char  *L1_ErrorString(L1_Error_t err);

/* 供 usart-x.c 回调使用 */
extern volatile uint8_t s_rx_byte;
void l1_parse_byte(uint8_t ch);

/* 外部实例 (兼容旧代码) */
extern Modbus_Instance_t g_l1_modbus_inst;

#endif /* __L1_LASER_H */