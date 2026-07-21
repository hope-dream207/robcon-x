#ifndef R2006_H
#define R2006_H
#include "struct_typedef.h"
#include "pid.h"
#include "ecpid.h"
#include "CAN_receive.h"
#include "mytask.h"
#include "arm.h"

typedef struct
{
   float R2006_ecpid[4][3];
   float R2006_pid[4][3];
   ECPidTypeDef R2006_position_motor[4];
   pid_type_def R2006_speed_motor[4];
   motor_measure_t R2006_motor_measure[4];
   float R2006_Angel_set[4];
   float R2006_Speed_set[4];
   float R2006_Angel_get[4];
   float R2006_Speed_get[4];
   float current_out[4];

} R2006_TypeDef;

void R2006_Init(void);
void R2006_Control(void);
void R2006_Arm_Control(void);

#endif
