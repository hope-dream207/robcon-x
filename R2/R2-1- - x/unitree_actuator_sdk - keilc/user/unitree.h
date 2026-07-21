#ifndef __UNITREE_H__
#define __UNITREE_H__

#include "gom_protocol.h"

#define MOTOR_NUM 3

#ifdef __cplusplus
extern "C" {
#endif

extern MotorCmd_t cmd[MOTOR_NUM];
extern MotorData_t data[MOTOR_NUM];
extern uint8_t Temp_buffer[64];  /* DMA 接收缓冲 (定义于 main.c) */

void Unitree_Init(void);
void Unitree_SetMotor(uint8_t motor_id, float q, float dq, float tau, float kp, float kd);
void Unitree_Control(void);
void Unitree_OnUart6Rx(const uint8_t *buf, size_t len);
float Unitree_GetJointAngle(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif