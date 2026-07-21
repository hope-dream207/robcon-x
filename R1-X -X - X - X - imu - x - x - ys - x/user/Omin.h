#ifndef OMIN_H
#define OMIN_H
#include "CAN_receive.h"
#include "ecpid.h"
#include "pid.h"
#include <stdint.h>


#define SQRT2 1.414f                    // 根号2
#define PIX 3.14159265f                  // 圆周率
#define OMIN_WHEEL_LA 0.65f              // 长，单位：米
#define OMIN_WHEEL_LB 0.65f              // 宽，单位：米
#define OMIN_WHEEL_RL 0.05f             // 全向轮轮距，单位：米
#define OMIN_WHEEL_Rr 0.08f              // 全向轮半径，单位：米

static const float w_L = (OMIN_WHEEL_LA + OMIN_WHEEL_LB + OMIN_WHEEL_RL) / 2.0f; // 轮子到中心的距离

typedef struct {
  float PID_Omni[4][3]; // 全向轮PID参数
  float PID_CHASSIS_Vx[3];
  float PID_CHASSIS_Vy[3];
  float PID_CHASSIS_Vw[3];
  pid_type_def Omni_pid_struct[4]; // 全向轮PID结构体
  pid_type_def chassis_vx;
  pid_type_def chassis_vy;
  pid_type_def chassis_vw;

  motor_measure_t Omni_motor_measure[4]; // 全向轮电机测量数据

  float wheel_w[4];       // 轮子角速度，单位：rad/s
  float wheel_rpm_set[4]; // 轮子转速，单位：rpm
  float current_cmd[4];   // 全向轮电流命令
  int16_t current_out[4]; // 全向轮电机电流输出命令
  float w[4];             // 正解算中使用的轮子角速度
  float v[4];             // 正解算中使用的轮子线速度
} Chassis_TypeDef;

typedef struct { // 逆
  float f_vx;    // 前后速度
  float f_vy;    // 左右速度
  float f_vw;    // 旋转速度
  float f_z;  // 修补 
  // 正
  float vx; // 前后速度
  float vy; // 左右速度
  float vw; // 旋转速度
  // 最终
  float final_vx;
  float final_vy;
  float final_vw;
} Chassis_Speed;

void Omni_Init(void);
void Omni_inverse_calc(void); // 逆解算：车体速度 -> 轮速
void Omni_forward_calc(void); // 正解算：轮速 -> 车体速度
void Omni_calc(void);         // 命令输入 + 状态融合输出

#endif