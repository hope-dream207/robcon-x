#include "grip_turnd.h"
#include "CAN_receive.h"
#include "Omin.h"
#include "ecpid.h"
#include "main.h"
#include "mytask.h"
#include "pid.h"
#include "sbus.h"
#include "usart.h"
#include <math.h>
#include <stdint.h>

#define GT_GFA_DEADBAND     0.03f
#define GT_CLAW_POS_MIN   105.0f
#define GT_CLAW_POS_MAX   305.0f
#define GT_CLAW_FAST_TARGET 210.0f
// #define GT_TURN_STEP_MAX    4.0f
// #define GT_TURN_ON          200
// #define GT_TURN_OFF         150
extern C_AND_GTLS c_and_gtls;
extern HOW_TO_CONTROL how ;
GT_TypeDef GT;//305,105
extern GT_Djl6020 GT_g;
extern int16_t g_sbus_channels[18];

void GT_Init() {

  // GTS ECPID 参数初始化
  // 电机0 PID参数（可自定义不同值），爪子,6020
	GT.ECPID_GT[0][0] = 25.0f; // 电机0 P
	GT.ECPID_GT[0][1] = 0.06f;  // 电机0 I 
	GT.ECPID_GT[0][2] = 2.0f;  // 电机0 D

	GT.PID_GT[0][0] = 80.0f; // 电机0 P 
	GT.PID_GT[0][1] = 0.02f;  // 电机0 I 
	GT.PID_GT[0][2] = 0.5f;  // 电机0 D
	

  // 电机1 PID参数（可自定义不同值）,转盘
  GT.ECPID_GT[1][0] = 13.0f; // 电机1 P
  GT.ECPID_GT[1][1] = 0.05f;  // 电机1 I 
  GT.ECPID_GT[1][2] = 0.1f;  // 电机1 D

  GT.PID_GT[1][0] = 10.0f; // 电机1 P 
  GT.PID_GT[1][1] = 0.08f; // 电机1 I 
  GT.PID_GT[1][2] = 0.3f; // 电机1 D

  ECPID_init(&GT.GT_ecpid_struct[0], ECPID_POSITION, GT.ECPID_GT[0], 600, 20.0f, 0.5f, 60.0f, 0.3f, 0.0f);

  PID_init(&GT.GT_pid_struct[0], PID_POSITION, GT.PID_GT[0], 15000.0f, 4000.0f, 10.0f, 50.0f, 0.5f, 0.5f);

  ECPID_init(&GT.GT_ecpid_struct[1], ECPID_POSITION, GT.ECPID_GT[1], 12000,
             1000.0f, 0.5f, 80.0f, 0.3f, 0.0f);
  // 转盘电机 M2006，C610持续最大10A → max_out=10000对应10A，足够堵转出力
  PID_init(&GT.GT_pid_struct[1], PID_POSITION, GT.PID_GT[1], 10000.0f, 2000.0f,
           80.0f, 1000.0f, 0.3f, 0.1f);
  
  CAN_cmd_GT_motor(0, 0); // 控制电机0、1
}

void GT_Control(uint8_t is_active) {
  C_AND_GTLS_change();
  //************************* 设定 **********************************//
  static uint8_t GT_first_run = 1;
  static float GT_init_angle[2] = {0.0f};       // 记录上电初始机械零位
  static uint8_t turntable_on = 0;
  GT.GT_Angel_get[0] = GT.GT_motor_measure[0].real_angle3;
  GT.GT_Angel_get[1] = GT.GT_motor_measure[1].real_angle1;
  GT.GT_Speed_get[0] = GT.GT_motor_measure[0].speed_rpm;
  GT.GT_Speed_get[1] = GT.GT_motor_measure[1].speed_rpm;
  
  // 首次运行获取零点
  if (GT_first_run) {
    GT_init_angle[0] = 298.87f;//45.5
    GT_init_angle[1] = GT.GT_Angel_get[1];
    GT_first_run = 0;
  }

  //  确定最终希望到达的绝对终点位置
  float final_target_pos[2] = {0.0f};
  final_target_pos[0] = GT_init_angle[0] + GT.GT_Angel_set[0];
  final_target_pos[1] = GT_init_angle[1] + GT.GT_Angel_set[1];
  
  //************************** 控制 **********************************/
  if (c_and_gtls == GTLS_MOVE ) {
    /**
     * @brief       6020电机(-88度),298.87除，到208,反是392
     */
    float claw_pos = final_target_pos[0];
    // 根据爪子位置调整速度增益(距离线性衰减)
    float claw_speed_factor = 1.0f;
   if(is_active){
    if (claw_pos >= 202.0f && claw_pos <= 214.0f) {
      float dist = fabsf(claw_pos - 208.0f) / 6.0f; // 中心208，半宽6
      claw_speed_factor = 0.05f + 0.5f * dist;       // 中心0.1，边界1.0
    } else if (claw_pos >= GT_init_angle[0] - 6.0f &&
               claw_pos <= GT_init_angle[0] + 6.0f) {
      float dist = fabsf(claw_pos - GT_init_angle[0]) / 6.0f;
      claw_speed_factor = 0.05f + 0.5f * dist;
    } else if (claw_pos >= 386.0f && claw_pos <= 398.0f) {
      float dist = fabsf(claw_pos - 392.0f) / 6.0f; // 中心392，半宽6
      claw_speed_factor = 0.05f + 0.5f * dist;
    }
    GT.GT_Angel_set[0] -= GT_g.f_Gfa * claw_speed_factor;
 }
  GT.GT_Angel_set[0] = fmaxf(GT_CLAW_POS_MIN - GT_init_angle[0],
                              fminf(GT_CLAW_POS_MAX - GT_init_angle[0], GT.GT_Angel_set[0]));
  final_target_pos[0] = GT_init_angle[0] + GT.GT_Angel_set[0];
}
  //**************************** 输出************************************//
  for (int i = 0; i < 2; i++) {
    // 位置环目标：
    GT.GT_Speed_set[i] = ECPID_Calc(&GT.GT_ecpid_struct[i], GT.GT_Angel_get[i],
                                    final_target_pos[i]);
    // 2. 速度环目标：
    GT.GT_current_out[i] =
        PID_calc(&GT.GT_pid_struct[i], GT.GT_Speed_get[i], GT.GT_Speed_set[i]);
  }

  // 发送CAN指令控制电机
  CAN_cmd_GT_motor(GT.GT_current_out[0], 0);
}