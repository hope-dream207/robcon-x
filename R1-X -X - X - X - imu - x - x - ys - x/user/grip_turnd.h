#ifndef GRIP_TURND_H
#define GRIP_TURND_H
#include "struct_typedef.h"
#include "pid.h"
#include "ecpid.h"
#include "CAN_receive.h"
#include "mytask.h"

typedef struct
{  //前1R6020，gripper，后2R2006： turn，sucker 
   float PID_GT[2][3];//  P I D 参数数组 position
   float ECPID_GT[2][3];//  P I D 参数数组 speed
   ECPidTypeDef GT_ecpid_struct[2];   //  电机位置环pid结构体
   pid_type_def GT_pid_struct[2]; //  电机速度环pid结构体
   motor_measure_t GT_motor_measure[2]; //  电机测量值指针数组
   float GT_Angel_set[2]; //  角度值
   float GT_Speed_set[2]; //  速度值
   float GT_Angel_get[2]; 
   float GT_Speed_get[2]; 
   float GT_current_out[2]; // 电机电流输出值数组
} GT_TypeDef;
typedef struct
{
   float f_Gfa;
} GT_Djl6020;

void GT_Init(void);
void GT_Control(uint8_t is_active);
#endif