#include "R2006.h"
#include "CAN_receive.h"
#include "ecpid.h"
#include "main.h"
#include "mytask.h"
#include "pid.h"
#include "usart.h"
#include "arm.h"
#include <math.h>
#include <stdint.h>

R2006_TypeDef R2006;
extern CAN_HandleTypeDef hcan2;
extern ROTATE_CONTROL rotate_run;
extern ARM_TO ARM_to;
void R2006_Init() {

  // R2006 ECPID 参数初始化
  // 电机0 PID参数，爪子
  R2006.R2006_ecpid[0][0] = 20.0f; // 电机0 P
  R2006.R2006_ecpid[0][1] = 0.0f;  // 电机0 I
  R2006.R2006_ecpid[0][2] = 0.1f;  // 电机0 D

  R2006.R2006_pid[0][0] = 18.0f; // 电机0 P
  R2006.R2006_pid[0][1] = 0.25f; // 电机0 I
  R2006.R2006_pid[0][2] = 0.1f;  // 电机0 D

  // 电机1 PID参数，转盘
  R2006.R2006_ecpid[1][0] = 18.0f; // 电机1 位置P，
  R2006.R2006_ecpid[1][1] = 0.0f;  // 电机1 位置I
  R2006.R2006_ecpid[1][2] = 0.0f;  // 电机1 位置D

  R2006.R2006_pid[1][0] = 15.0f; // 电机1 速度P，
  R2006.R2006_pid[1][1] = 0.0f;  // 电机1 速度I 
  R2006.R2006_pid[1][2] = 0.0f;  // 电机1 速度D 

  // 电机2 PID参数 转盘
  R2006.R2006_ecpid[2][0] = 18.0f;
  R2006.R2006_ecpid[2][1] = 0.0f;
  R2006.R2006_ecpid[2][2] = 0.0f;

  R2006.R2006_pid[2][0] = 15.0f; // 电机2 速度P，
  R2006.R2006_pid[2][1] = 0.0f;
  R2006.R2006_pid[2][2] = 0.0f;

  // 电机3 PID参数，转盘
  R2006.R2006_ecpid[3][0] = 18.0f;
  R2006.R2006_ecpid[3][1] = 0.0f;
  R2006.R2006_ecpid[3][2] = 0.0f;

  R2006.R2006_pid[3][0] = 15.0f;
  R2006.R2006_pid[3][1] = 0.0f;
  R2006.R2006_pid[3][2] = 0.0f;

  // R2006 电机位置环 ECPID 结构体初始化
  for (int i = 0; i < 4; i++) {
    ECPID_init(&R2006.R2006_position_motor[i], ECPID_POSITION,
               R2006.R2006_ecpid[i], 10000.0f, 2000.0f, 0.15f, 8.0f, 0.8f,
               0.0f);
    PID_init(&R2006.R2006_speed_motor[i], PID_POSITION, R2006.R2006_pid[i],
             10000.0f, 2000.0f, 5.0f, 250.0f, 0.45f, 0.0f);
  }
  CAN_cmd_R2006_motor(0, 0, 0, 0); // 控制电机0、1和2
}

void R2006_Control() {
  static uint8_t R2006_first_run = 1;
  static float R2006_init_angle[3] = {0.0f}; // 记录上电初始机械零位

  R2006.R2006_Angel_get[0] = R2006.R2006_motor_measure[0].real_angle1;
  R2006.R2006_Speed_get[0] = R2006.R2006_motor_measure[0].speed_rpm;

  R2006.R2006_Angel_get[1] = R2006.R2006_motor_measure[1].real_angle1;
  R2006.R2006_Speed_get[1] = R2006.R2006_motor_measure[1].speed_rpm;
  static float smoothed_target_pos[3] = {0.0f}; // 影子平滑目标位置
                                                // 首次运行获取零点
  if (R2006_first_run) {
    R2006_init_angle[0] = R2006.R2006_Angel_get[0];
    R2006_init_angle[1] = R2006.R2006_Angel_get[1];
    R2006_init_angle[2] = R2006.R2006_Angel_get[2];
    smoothed_target_pos[0] =
        R2006_init_angle[0]; // 初始化时，平滑目标对齐原始位置
    smoothed_target_pos[1] =
        R2006_init_angle[1]; // 初始化时，平滑目标对齐原始位置
    smoothed_target_pos[2] =
        R2006_init_angle[2]; // 初始化时，平滑目标对齐原始位置
    R2006_first_run = 0;
  }
  if (rotate_run == rotate_turn) {
    R2006.R2006_Angel_set[0] = -366.7f; // 393,366
	 R2006.R2006_Angel_set[1] = 0.0f;	  
  }
  else{
  R2006.R2006_Angel_set[1] = 760.0f;} // 开始就转盘转
  //  确定最终希望到达的绝对终点位置
  float final_target_pos[3] = {0.0f};
  final_target_pos[0] = R2006_init_angle[0] + R2006.R2006_Angel_set[0];
  final_target_pos[1] = R2006_init_angle[1] + R2006.R2006_Angel_set[1];
  smoothed_target_pos[0] +=
      (final_target_pos[0] - smoothed_target_pos[0]) * 0.35f;
  smoothed_target_pos[1] +=
      (final_target_pos[1] - smoothed_target_pos[1]) * 0.35f;
  for (int i = 0; i < 2; i++) {
    // 位置环追踪的是平滑后的影子目标
    R2006.R2006_Speed_set[i] =
        ECPID_Calc(&R2006.R2006_position_motor[i], R2006.R2006_Angel_get[i],
                   smoothed_target_pos[i]);
    // 2. 速度环目标：
    R2006.current_out[i] =
        PID_calc(&R2006.R2006_speed_motor[i], R2006.R2006_Speed_get[i],
                 R2006.R2006_Speed_set[i]);
  }

  // 发送CAN指令控制电机
  CAN_cmd_R2006_motor(R2006.current_out[0], R2006.current_out[1], 0, 0);
}

void R2006_Arm_Control(void) {
  static uint8_t R2006_arm_first_run = 1;
  static float R2006_arm_init_angle[2] = {0.0f};

  R2006.R2006_Angel_get[2] = R2006.R2006_motor_measure[2].real_angle1;
  R2006.R2006_Angel_get[3] = R2006.R2006_motor_measure[3].real_angle1;

  if (R2006_arm_first_run) {
    R2006_arm_init_angle[0] = R2006.R2006_Angel_get[2];
    R2006_arm_init_angle[1] = R2006.R2006_Angel_get[3];
    R2006_arm_first_run = 0;
  }

  float final_target_pos[2] = {0.0f};
  final_target_pos[0] = R2006_arm_init_angle[0] + ARM_to.set3;
  final_target_pos[1] = R2006_arm_init_angle[1] - ARM_to.set3;

  R2006.R2006_Speed_set[2] =
      ECPID_Calc(&R2006.R2006_position_motor[2], R2006.R2006_Angel_get[2],
                 final_target_pos[0]);
  R2006.current_out[2] = PID_calc(&R2006.R2006_speed_motor[2],
                                  R2006.R2006_motor_measure[2].speed_rpm,
                                  R2006.R2006_Speed_set[2]);

  R2006.R2006_Speed_set[3] =
      ECPID_Calc(&R2006.R2006_position_motor[3], R2006.R2006_Angel_get[3],
                 final_target_pos[1]);
  R2006.current_out[3] = PID_calc(&R2006.R2006_speed_motor[3],
                                  R2006.R2006_motor_measure[3].speed_rpm,
                                  R2006.R2006_Speed_set[3]);

  CAN_cmd_R2006_motor(0, 0, R2006.current_out[2], R2006.current_out[3]);
}
