#ifndef CONTORL_TWO_H
#define CONTORL_TWO_H
#include "CAN_receive.h"
#include "pid.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define SQRT2 1.414f // 根号2
#define WHEEL_LA1 0.55f              // 长，单位：米
#define WHEEL_LB 0.62f              // 宽，单位：米
#define M_WHEEL_RL 0.05f           // 麦轮轮距，单位：米0.24,0.31
#define M_WHEEL_Rr 0.076f            // 麦轮半径，单位：米
#define OMIN_WHEEL_RL 0.05f        // 全向轮轮距，单位：米
#define OMIN_WHEEL_Rr 0.076f         // 全向轮半径，单位：米
//重心靠前了
static const float w_ML1 =(WHEEL_LA1 + WHEEL_LB+M_WHEEL_RL ) / 2.0f; // 麦轮子到中心的距离
static const float w_ML2 =(WHEEL_LA1 + WHEEL_LB+M_WHEEL_RL ) / 2.0f; // 麦轮子到中心的距离
static const float w_OL = WHEEL_LB/ 2.0f+ OMIN_WHEEL_RL/2.0f+0.05f; // 全向轮子到中心的距离
extern int16_t chassis_3508_setspeed[4];
extern int16_t chassis_3508_setspeed2[2];

typedef struct {
  float PID_WHEEL[4][3];  // 麦轮 PID 参数
  float PID_WHEELO[2][3]; // 全向轮 PID 参数
  float PID_WHEEL_Vx[3];
  float PID_WHEEL_Vy[3];
  float PID_WHEEL_Vw[3];

  pid_type_def wheelM_pid_struct[4]; // 麦轮 PID
  pid_type_def wheelO_pid_struct[2]; // 全向轮 PID
  pid_type_def wheel_vx;
  pid_type_def wheel_vy;
  pid_type_def wheel_vw;

  motor_measure_t wheel_motor[4];  // 麦轮 3508 电机反馈数据
  motor_measure_t wheelO_motor[2]; // 全向轮 3508 电机反馈数据

  float wheel_Mw[4];       // 轮子角速度，单位：rad/s
  float wheel_Ow[2];       // 全向轮角速度，单位：rad/s
  float wheel_rpm_Mset[4]; // 麦轮转速，单位：rpm
  float wheel_rpm_Oset[2]; // 全向轮转速，单位：rpm
  int16_t current_outM[4]; // 麦轮电机电流输出命令
  int16_t current_outO[2]; // 全向轮电机电流输出命令
  float M_w[4];            // 正解算中使用的轮子角速度
  float M_v[4];            // 正解算中使用的轮子线速度
  float O_w[2];            // 正解算中使用的轮子角速度
  float O_v[2];            // 正解算中使用的轮子线速度

} Wheel_type;

typedef struct { // 逆
  float f_Mvx;   // 前后速度
  float f_Mvy;   // 左右速度
  float f_Mvw;   // 旋转速度
  float f_Ovx;   // 前后速度
  // float f_Ovy;  // 左右速度
  float f_Ovw; // 旋转速度
  float f_z;   // 零偏
  float f_l;   // 激光 
  // 正
  float Mvx; // 前后速度
  float Mvy; // 左右速度
  float Mvw; // 旋转速度
  float Ovx; // 前后速度
  // float Ovy;  // 左右速度
  float Ovw; // 旋转速度
  // 最终
  float final_Mvx;
  float final_Mvy;
  float final_Mvw;
  float final_Ovx;
  // float final_Ovy;
  float final_Ovw;
} Wheel_Speed;

extern Wheel_Speed f_speed;
extern Wheel_Speed speed;
extern Wheel_Speed final_speed;

void Chassis_init(void);       // 底盘开环初始化
void wheel_inverse_calc(void); // 轮子逆解算
void wheel_forward_calc(void); // 轮子正解算
void wheel_calc(void);         // 轮子控制计算

#endif
