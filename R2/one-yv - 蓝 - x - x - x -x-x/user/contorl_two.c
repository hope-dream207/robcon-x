/** 
  * @file contorl_two.c 
  * @brief 混合底盘控制：4个麦轮 + 2个全向轮 
  * - 4个麦轮（索引0-3）用于全向移动 
  * - 2个全向轮（索引0-1）用于额外的横向移动支持 
  * 
  * 电机分配： 
  * - 麦轮：4个电机通过CAN ID 0x200控制 
  * - 全向轮：2个电机通过CAN ID 0x1FF控制 
  */ 
 #include "CAN_receive.h" 
 #include "contorl_two.h" 
 #include "mytask.h" 
 #include "FreeRTOS.h"
 #include "task.h"
 #include <math.h> 
 
 #define xSTEP 0.5f 
 Wheel_type wheel; 
 Wheel_Speed f_speed; 
 Wheel_Speed speed; 
 Wheel_Speed final_speed; 
 extern Wheel_Speed chassis_cmd_speed; 
 extern HOW_TO_CONTROL how; 

 //左右反了
 static uint16_t startup_cnt = 0;
 static uint8_t  motor_feedback_received = 0;
 #define STARTUP_HOLD_MS  100   // 收到反馈后保持100周期不出力（200ms，等IMU/CAN收敛）
 
 static const float M_WHEEL_Rr_calib[4] = { 
//      M_WHEEL_Rr + 0.0085f, // 轮0: 前左 
      M_WHEEL_Rr+ 0.0025f,  // 轮0: 前左 
      M_WHEEL_Rr+ 0.0025f,  // 轮1: 前右 
//      M_WHEEL_Rr + 0.0068f, // 轮2: 后左 
     M_WHEEL_Rr+ 0.0030f,  // 轮2: 后左 
     M_WHEEL_Rr+ 0.0030f  // 轮3: 后右 
 }; 
 static float compare(float a , float b ,float c )  // a是最终目标值，b是当前渐变值，c是最大限制步长 
 { 
   float m = a  - b ; 
   if (m > c ) { 
     return b  + c ;  // 目标极大于当前值，正向以步长递增 
   } else if (m < - c ) { 
     return b  - c ;  // 目标极小于当前值，反向以步长递减 
   } else { 
     return a ;  // 差值在步长范围内，直接到位 
   } 
 } 
 //前左1右2 后左3右4 
 void Chassis_init() { 
   // wheel.PID_WHEEL[0][0] = 16.0f; 
   // wheel.PID_WHEEL[0][1] = 0.5f; 
   // wheel.PID_WHEEL[0][2] = 0.25f; 
 
 
   // wheel.PID_WHEEL[1][0] = 16.0f; 
   // wheel.PID_WHEEL[1][1] = 0.5f; 
   // wheel.PID_WHEEL[1][2] = 0.25f; 
   //重心靠前了 
   wheel.PID_WHEEL[0][0] = 14.5f; 
   wheel.PID_WHEEL[0][1] = 0.1f; 
   wheel.PID_WHEEL[0][2] = 0.25f; 
 
 
   wheel.PID_WHEEL[1][0] = 14.5f; 
   wheel.PID_WHEEL[1][1] = 0.1f; 
   wheel.PID_WHEEL[1][2] = 0.25f; 
 
 
   wheel.PID_WHEEL[2][0] = 14.5f; 
   wheel.PID_WHEEL[2][1] = 0.1f; 
   wheel.PID_WHEEL[2][2] = 0.25f; 
 
 
   wheel.PID_WHEEL[3][0] = 14.0f; 
   wheel.PID_WHEEL[3][1] = 0.1f; 
   wheel.PID_WHEEL[3][2] = 0.25f; 
 
 
   wheel.PID_WHEELO[0][0] = 14.25f; 
   wheel.PID_WHEELO[0][1] = 0.1f; 
   wheel.PID_WHEELO[0][2] = 0.25f; 
 
 
   wheel.PID_WHEELO[1][0] = 14.0f; 
   wheel.PID_WHEELO[1][1] = 0.1f; 
   wheel.PID_WHEELO[1][2] = 0.25f; 
 
 
   wheel.PID_WHEEL_Vx[0] = 5.0f; 
   wheel.PID_WHEEL_Vx[1] = 0.0f; 
   wheel.PID_WHEEL_Vx[2] = 0.5f; 
 
 
   wheel.PID_WHEEL_Vy[0] = 5.0f; 
   wheel.PID_WHEEL_Vy[1] = 0.0f; 
   wheel.PID_WHEEL_Vy[2] = 0.5f; 
 
 
   wheel.PID_WHEEL_Vw[0] = 5.0f; 
   wheel.PID_WHEEL_Vw[1] = 0.0f; 
   wheel.PID_WHEEL_Vw[2] = 0.5f; 
   
  //  wheel.PID_WHEEL[0][0] = 0.0f; 
  //  wheel.PID_WHEEL[0][1] = 0.0f; 
  //  wheel.PID_WHEEL[0][2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL[1][0] = 0.0f; 
  //  wheel.PID_WHEEL[1][1] = 0.0f; 
  //  wheel.PID_WHEEL[1][2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL[2][0] = 0.0f; 
  //  wheel.PID_WHEEL[2][1] = 0.0f; 
  //  wheel.PID_WHEEL[2][2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL[3][0] = 0.0f; 
  //  wheel.PID_WHEEL[3][1] = 0.0f; 
  //  wheel.PID_WHEEL[3][2] = 0.0f; 
 
 
  //  wheel.PID_WHEELO[0][0] = 0.0f; 
  //  wheel.PID_WHEELO[0][1] = 0.0f; 
  //  wheel.PID_WHEELO[0][2] = 0.0f; 
 
 
  //  wheel.PID_WHEELO[1][0] = 0.0f; 
  //  wheel.PID_WHEELO[1][1] = 0.0f; 
  //  wheel.PID_WHEELO[1][2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL_Vx[0] = 0.0f; 
  //  wheel.PID_WHEEL_Vx[1] = 0.0f; 
  //  wheel.PID_WHEEL_Vx[2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL_Vy[0] = 0.0f; 
  //  wheel.PID_WHEEL_Vy[1] = 0.0f; 
  //  wheel.PID_WHEEL_Vy[2] = 0.0f; 
 
 
  //  wheel.PID_WHEEL_Vw[0] = 0.0f; 
  //  wheel.PID_WHEEL_Vw[1] = 0.0f; 
  //  wheel.PID_WHEEL_Vw[2] = 0.0f; 

   // 统一采用 16384 等大参数限幅（清理了重复的被覆盖代码） 
   for (uint8_t i = 0; i < 4; i++) { 
     PID_init(&wheel.wheelM_pid_struct[i], PID_POSITION, wheel.PID_WHEEL[i], 
              16384, 8000, 30.0f, 800.0f, 0.4f, 0.00f); 
     PID_clear(&wheel.wheelM_pid_struct[i]); 
   } 
   for (uint8_t t = 0; t < 2; t++) { 
     PID_init(&wheel.wheelO_pid_struct[t], PID_POSITION, wheel.PID_WHEELO[t], 
              16384, 8000, 30.0f, 800.0f, 0.4f, 0.1f); 
     PID_clear(&wheel.wheelO_pid_struct[t]); 
   } 
   PID_init(&wheel.wheel_vx, PID_POSITION, wheel.PID_WHEEL_Vx, 0.3f, 0.2f, 0.05f, 
            1.0f, 0.45f, 0.0f); 
   PID_init(&wheel.wheel_vy, PID_POSITION, wheel.PID_WHEEL_Vy, 0.3f, 0.2f, 0.05f, 
            1.0f, 0.45f, 0.0f); 
   PID_init(&wheel.wheel_vw, PID_POSITION, wheel.PID_WHEEL_Vw, 0.3f, 0.2f, 0.05f, 
            1.0f, 0.45f, 0.0f); 
   PID_clear(&wheel.wheel_vx); 
   PID_clear(&wheel.wheel_vy); 
   PID_clear(&wheel.wheel_vw); 
 
   // 上电软启动计数清零
   startup_cnt = 0;
 
   CAN_cmd_chassis_wheelM(0, 0, 0, 0); 
   CAN_cmd_chassis_wheelO(0, 0); 
 } 
 
 
 // M麦轮 O全向轮的逆解算 
 void wheel_inverse_calc(void) { 
   float k_z = 1.1f; 
   taskENTER_CRITICAL();
   float f_z_val = chassis_cmd_speed.f_z;
   taskEXIT_CRITICAL();
   wheel.wheel_Mw[0] = ( f_speed.f_Mvx + f_speed.f_Mvy +  (f_speed.f_Mvw + k_z*f_z_val) * w_ML1) / 
                         M_WHEEL_Rr_calib[0];  // 前左 
   wheel.wheel_Mw[1] = (-f_speed.f_Mvx + f_speed.f_Mvy +  (f_speed.f_Mvw + k_z*f_z_val) * w_ML1) / 
                       M_WHEEL_Rr_calib[1];  // 前右 
   wheel.wheel_Mw[2] = ( f_speed.f_Mvx - f_speed.f_Mvy +  (f_speed.f_Mvw + k_z*f_z_val) * w_ML2) / 
                       M_WHEEL_Rr_calib[2];  // 后左 
   wheel.wheel_Mw[3] = (-f_speed.f_Mvx - f_speed.f_Mvy +  (f_speed.f_Mvw + k_z*f_z_val) * w_ML2) / 
                       M_WHEEL_Rr_calib[3];  // 后右 
   wheel.wheel_Ow[0] =
      ( f_speed.f_Ovx - (-f_speed.f_Ovw - f_z_val) * w_OL) / OMIN_WHEEL_Rr / SQRT2;
   wheel.wheel_Ow[1] =
      (-f_speed.f_Ovx - (-f_speed.f_Ovw - f_z_val) * w_OL) / (OMIN_WHEEL_Rr+0.005f)/ SQRT2 ; 
 
 
   // 转换为轮子转速并且乘以减速比 18.1935 转换为电机转速 
   for (uint8_t i = 0; i < 4; i++) { 
     wheel.wheel_rpm_Mset[i] = wheel.wheel_Mw[i] * CONV; 
   } 
   for (uint8_t i = 0; i < 2; i++) { 
     wheel.wheel_rpm_Oset[i] = wheel.wheel_Ow[i] * CONV; 
   } 
   // 计算 
   for (uint8_t i = 0; i < 4; i++) { 
     wheel.current_outM[i] = 
         PID_calc(&wheel.wheelM_pid_struct[i], wheel.wheel_motor[i].speed_rpm, 
                  wheel.wheel_rpm_Mset[i] * 18.1935f); 
   } 
   for (uint8_t i = 0; i < 2; i++) { 
     wheel.current_outO[i] = 
         PID_calc(&wheel.wheelO_pid_struct[i], wheel.wheelO_motor[i].speed_rpm, 
                  wheel.wheel_rpm_Oset[i] * 18.1935f); 
   } 
 }
 
 
 // M麦轮 O全向轮的正解算 
 void wheel_forward_calc(void) { 
   for (uint8_t i = 0; i < 4; i++) { 
     wheel.M_w[i] = wheel.wheel_motor[i].speed_rpm / 18.1935f / CONV; 
     wheel.M_v[i] = wheel.M_w[i] * M_WHEEL_Rr_calib[i]; 
   } 
   static const float O_WHEEL_Rr_calib[2] = { OMIN_WHEEL_Rr, OMIN_WHEEL_Rr+0.005f  };
   for (uint8_t t = 0; t < 2; t++) { 
     wheel.O_w[t] = wheel.wheelO_motor[t].speed_rpm / 18.1935f / CONV; 
     wheel.O_v[t] = wheel.O_w[t] * O_WHEEL_Rr_calib[t]; 
   } 
   speed.Mvx = ( wheel.M_v[0] - wheel.M_v[1] + wheel.M_v[2] - wheel.M_v[3]) / 4.0f; 
   speed.Mvy = (wheel.M_v[0] + wheel.M_v[1] - wheel.M_v[2] - wheel.M_v[3]) / 4.0f; 
   speed.Mvw = (wheel.M_v[0] + wheel.M_v[1] + wheel.M_v[2] + wheel.M_v[3]) / (4.0f * w_ML1); 
   speed.Ovx = (wheel.O_v[0] - wheel.O_v[1]) / 2.0f * SQRT2; 
   speed.Ovw = (wheel.O_v[0] + wheel.O_v[1]) / 2.0f / w_OL ; 
 } 
 
 
 // M麦轮 O全向轮的最终解算 
 void wheel_calc(void) { 
  // 上电软启动,不然上电猛冲
  if (!motor_feedback_received) {
    if (wheel.wheel_motor[0].msg_cnt > 1000 || wheel.wheelO_motor[0].speed_rpm > 0) {
      motor_feedback_received = 1;
    }
  }
  if (!motor_feedback_received || startup_cnt < STARTUP_HOLD_MS) {
    if (motor_feedback_received) startup_cnt++;
     for (uint8_t i = 0; i < 4; i++) PID_clear(&wheel.wheelM_pid_struct[i]);
     for (uint8_t i = 0; i < 2; i++) PID_clear(&wheel.wheelO_pid_struct[i]);
     PID_clear(&wheel.wheel_vx);
     PID_clear(&wheel.wheel_vy);
     PID_clear(&wheel.wheel_vw);
     f_speed.f_Mvx = 0.0f;
     f_speed.f_Mvy = 0.0f;
     f_speed.f_Mvw = 0.0f;
     f_speed.f_Ovx = 0.0f;
     f_speed.f_Ovw = 0.0f;
     CAN_cmd_chassis_wheelM(0, 0, 0, 0);
     CAN_cmd_chassis_wheelO(0, 0);
     return;
   }
   // 目标速度：只读取指令，不覆盖 f_speed（f_speed 保留上一拍渐变值，供 compare 渐变使用）
   float target_Mvx, target_Mvy, target_Mvw, target_Ovx, target_Ovw;
   taskENTER_CRITICAL();
   target_Mvx = chassis_cmd_speed.f_Mvx;
   target_Mvy = chassis_cmd_speed.f_Mvy;
   target_Mvw = chassis_cmd_speed.f_Mvw;
   target_Ovx = chassis_cmd_speed.f_Ovx;
   target_Ovw = chassis_cmd_speed.f_Ovw;
   taskEXIT_CRITICAL(); 
 
   wheel_forward_calc();  // 先计算当前状态速度，供融合使用 
     float k1 = 0.2f; 
     float k2 = 0.2f; 
     float wheel_vx_e = PID_calc(&wheel.wheel_vx, speed.Mvx, target_Mvx); 
     float wheel_vy_e = PID_calc(&wheel.wheel_vy, speed.Mvy, target_Mvy); 
     float wheel_vw_e = PID_calc(&wheel.wheel_vw, speed.Mvw, target_Mvw); 
     // 麦轮速度误差计算 
     final_speed.final_Mvx = (1 - k1) * target_Mvx + k1 * wheel_vx_e; 
     final_speed.final_Mvy = (1 - k1) * target_Mvy + k1 * wheel_vy_e;   
     // 全向轮速度误差计算 
     final_speed.final_Ovx = (1 - k2) * target_Ovx + k2 * wheel_vx_e; 
     final_speed.final_Ovw = (1 - k2) * target_Ovw + k2 * wheel_vw_e; 
 
 
     // 渐变：f_speed 保留上一拍值，向 final_speed 逐步逼近（每周期最多 xSTEP）
     f_speed.f_Mvx = compare(final_speed.final_Mvx, f_speed.f_Mvx, xSTEP); 
     f_speed.f_Mvy = compare(final_speed.final_Mvy, f_speed.f_Mvy, xSTEP); 
     f_speed.f_Mvw = compare(final_speed.final_Mvw, f_speed.f_Mvw, xSTEP); 
     f_speed.f_Ovx = compare(final_speed.final_Ovx, f_speed.f_Ovx, xSTEP); 
     f_speed.f_Ovw = compare(final_speed.final_Ovw, f_speed.f_Ovw, xSTEP); 
 
   wheel_inverse_calc();  // 根据融合后的最终速度计算轮子命令 
   if(how == TWO){ 
     CAN_cmd_chassis_wheelM(wheel.current_outM[0]*1.35f, wheel.current_outM[1]*1.35f, 
                          wheel.current_outM[2]*1.35f, wheel.current_outM[3]*1.35f); 


     CAN_cmd_chassis_wheelO(wheel.current_outO[0]*1.35f, wheel.current_outO[1]*1.35f); 
   } 
   else { 
     CAN_cmd_chassis_wheelM(wheel.current_outM[0], wheel.current_outM[1], 
                          wheel.current_outM[2], wheel.current_outM[3]); 


     CAN_cmd_chassis_wheelO(wheel.current_outO[0], wheel.current_outO[1]); 
   } 
}