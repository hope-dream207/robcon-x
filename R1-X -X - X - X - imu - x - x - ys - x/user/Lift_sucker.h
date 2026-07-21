#ifndef LIFT_SUCKER_H
#define LIFT_SUCKER_H

#include "pid.h"
#include "ecpid.h"
#include "CAN_receive.h"

typedef struct
{
  pid_type_def lift_pid[2]; // 电机速度环pid结构体
  ECPidTypeDef lift_ecpid[2]; // 电机位置环pid结构体
  pid_type_def sucker_pid; 
  ECPidTypeDef sucker_ecpid; 
  
	float PID_LIFT[2][3];
	float ECPID_LIFT[2][3];
  float PID_sucker[3];
	float ECPID_sucker[3];

  motor_measure_t lift_su_motor_measure[3]; // 电机测量值数组
  float L_a[2];       //升降电机角度，单位：rad/s
  float L_rpm_Mset[2]; // 升降电机转速，单位：rpm
  float su_a;       
  float su_rpm_Mset; 
  int16_t current_outL[2]; // 升降电机电流输出命令
  int16_t current_outsu; 
}Lift_su_type;

typedef struct { 
  // 逆   
    // float f_Lfv;  
    // float f_sufv;
    float f_Lfa;  
    float f_sufa;   
  // 最终
  float final_Lfa;
  float final_sufa;
} Lift_su_Speed;

void Lift_su_Init(void);
void Lift_su_Reset(void);
void Lift_su_Control(uint8_t is_active);

#endif // LIFT_SUCKER_H
