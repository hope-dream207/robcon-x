#include "Omin.h"
#include "CAN_receive.h"
#include <math.h>
#include <string.h>
#include "mytask.h"

// 运动学预计算常量，避免运行时除法
#define RPM_TO_MOTOR      18.1935f  // 3508电机转速转换比
#define INV_RPM_TO_WHEEL  (1.0f / (RPM_TO_MOTOR * OMIN_CONV))         // rpm → 轮角速度
#define FWD_DIV_4SQRT2    (1.0f / (4.0f * SQRT2))                     // 正解算: vx/vy 系数
#define FWD_DIV_4WLSQRT2  (1.0f / (4.0f * w_L * SQRT2))              // 正解算: vw 系数

#define FEEDFORWARD_MIN    0.18f      // 前馈控制最小值
#define FEEDFORWARD_MAX    0.35f      // 前馈控制最大值 
#define FEEDFORWARD_ERR_GAIN 0.12f    // 前馈误差增益


#define SPEED_LPF_ALPHA    0.20f//步长
#define MAX_WHEEL_RPM      6000.0f//限速

Chassis_TypeDef chassis;
extern Chassis_Speed f_speed;
Chassis_Speed speed;
Chassis_Speed final_speed;
static Chassis_Speed speed_filt;
static float ramp_vx = 0.0f;
static float ramp_vy = 0.0f;
static float ramp_vw = 0.0f;

static float clampf(float x, float min_v, float max_v) {
  return fmaxf(fminf(x, max_v), min_v);
}


void Omni_Init() {
//  memset(&chassis, 0, sizeof(chassis));
  
  chassis.PID_Omni[0][0] = 15.0f; chassis.PID_Omni[0][1] = 0.3f; chassis.PID_Omni[0][2] = 0.5f;
  chassis.PID_Omni[1][0] = 15.0f; chassis.PID_Omni[1][1] = 0.3f; chassis.PID_Omni[1][2] = 0.5f;
  chassis.PID_Omni[2][0] = 15.0f; chassis.PID_Omni[2][1] = 0.3f; chassis.PID_Omni[2][2] = 0.5f;
  chassis.PID_Omni[3][0] = 15.0f; chassis.PID_Omni[3][1] = 0.3f; chassis.PID_Omni[3][2] = 0.5f;

  chassis.PID_CHASSIS_Vx[0] = 3.0f;  chassis.PID_CHASSIS_Vx[1] = 0.08f; chassis.PID_CHASSIS_Vx[2] = 0.08f;
  chassis.PID_CHASSIS_Vy[0] = 3.0f;  chassis.PID_CHASSIS_Vy[1] = 0.08f; chassis.PID_CHASSIS_Vy[2] = 0.08f;
  chassis.PID_CHASSIS_Vw[0] = 2.5f;  chassis.PID_CHASSIS_Vw[1] = 0.06f; chassis.PID_CHASSIS_Vw[2] = 0.06f;

  for (uint8_t i = 0; i < 4; i++) {
    PID_init(&chassis.Omni_pid_struct[i], PID_POSITION, chassis.PID_Omni[i],
             16000, 1500, 80.0f, 1000.0f, 0.5f, 0.05f);
    PID_clear(&chassis.Omni_pid_struct[i]);
  }
  PID_init(&chassis.chassis_vx, PID_POSITION, chassis.PID_CHASSIS_Vx, 600, 150, 8.5f,  200.0f, 0.25f, 0.1f);
  PID_init(&chassis.chassis_vy, PID_POSITION, chassis.PID_CHASSIS_Vy, 600, 150, 8.5f,  200.0f, 0.25f, 0.1f);
  PID_init(&chassis.chassis_vw, PID_POSITION, chassis.PID_CHASSIS_Vw, 500, 120, 10.5f, 200.0f, 0.25f, 0.1f);
  PID_clear(&chassis.chassis_vx);
  PID_clear(&chassis.chassis_vy);
  PID_clear(&chassis.chassis_vw);
  CAN_cmd_chassis(0, 0, 0, 0);
}

// 逆解算：车体速度 → 各轮转速 → PID → 电机电流
void Omni_inverse_calc(void) {
  const float vw_wL = final_speed.final_vw * w_L;  // 公共子表达式，4轮共用
  taskENTER_CRITICAL();
  float f_z_val = -f_speed.f_z;
  taskEXIT_CRITICAL();
  chassis.wheel_w[0] = (( final_speed.final_vx + final_speed.final_vy)* SQRT2 - (vw_wL + f_z_val))  / (OMIN_WHEEL_Rr);
  chassis.wheel_w[1] = ((-final_speed.final_vx + final_speed.final_vy)* SQRT2 - (vw_wL + f_z_val))  / (OMIN_WHEEL_Rr);
  chassis.wheel_w[2] = (( final_speed.final_vx - final_speed.final_vy)* SQRT2 - (vw_wL + f_z_val))  / (OMIN_WHEEL_Rr);
  chassis.wheel_w[3] = ((-final_speed.final_vx - final_speed.final_vy)* SQRT2 - (vw_wL + f_z_val))  / (OMIN_WHEEL_Rr);

  float max_rpm = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    chassis.wheel_rpm_set[i] = chassis.wheel_w[i] * OMIN_CONV;
    const float abs_rpm = fabsf(chassis.wheel_rpm_set[i]);
    if (abs_rpm > max_rpm) {
      max_rpm = abs_rpm;
    }
  }

  if (max_rpm > MAX_WHEEL_RPM) {
    const float scale = MAX_WHEEL_RPM / max_rpm;
    for (uint8_t i = 0; i < 4; i++) {
      chassis.wheel_rpm_set[i] *= scale;
    }
  }

  for (uint8_t i = 0; i < 4; i++) {
    chassis.current_cmd[i] =
        PID_calc(&chassis.Omni_pid_struct[i],
                 (fp32)chassis.Omni_motor_measure[i].speed_rpm,
                 chassis.wheel_rpm_set[i] * RPM_TO_MOTOR);
    chassis.current_out[i] = (int16_t)chassis.current_cmd[i];
  }
}

// 正解算：电机转速 → 各轮线速度 → 车体速度
void Omni_forward_calc(void) {
  for (uint8_t i = 0; i < 4; i++) {
    chassis.w[i] = chassis.Omni_motor_measure[i].speed_rpm * INV_RPM_TO_WHEEL;
    chassis.v[i] = chassis.w[i] * OMIN_WHEEL_Rr;
  }
  speed.vx = (chassis.v[0] - chassis.v[1] + chassis.v[2] - chassis.v[3]) * FWD_DIV_4SQRT2;
  speed.vy = (chassis.v[0] + chassis.v[1] - chassis.v[2] - chassis.v[3]) * FWD_DIV_4SQRT2;
  speed.vw = (-chassis.v[0] - chassis.v[1] - chassis.v[2] - chassis.v[3]) * FWD_DIV_4WLSQRT2;

  speed_filt.vx += SPEED_LPF_ALPHA * (speed.vx - speed_filt.vx);
  speed_filt.vy += SPEED_LPF_ALPHA * (speed.vy - speed_filt.vy);
  speed_filt.vw += SPEED_LPF_ALPHA * (speed.vw - speed_filt.vw);
}

// 最终解算：前馈+反馈融合 → 逆解算 → 下发CAN报文
void Omni_calc(void) {
  Omni_forward_calc();

  float cmd_vx = f_speed.f_vx;
  float cmd_vy = f_speed.f_vy;
  float cmd_vw = f_speed.f_vw;

  ramp_vx += clampf(cmd_vx - ramp_vx, -0.08f, 0.08f);
  ramp_vy += clampf(cmd_vy - ramp_vy, -0.08f, 0.08f);
  ramp_vw += clampf(cmd_vw - ramp_vw, -0.05f, 0.05f);


  float chassis_vx_e = PID_calc(&chassis.chassis_vx, speed_filt.vx, ramp_vx);
  float chassis_vy_e = PID_calc(&chassis.chassis_vy, speed_filt.vy, ramp_vy);
  float chassis_vw_e = PID_calc(&chassis.chassis_vw, speed_filt.vw, ramp_vw);

  const float err_mag = fabsf(ramp_vx - speed_filt.vx)
                      + fabsf(ramp_vy - speed_filt.vy)
                      + 0.3f * fabsf(ramp_vw - speed_filt.vw);
  const float k = clampf(FEEDFORWARD_MIN + FEEDFORWARD_ERR_GAIN * err_mag,
                         FEEDFORWARD_MIN, FEEDFORWARD_MAX);

  // 前馈+反馈融合: (1-k)*cmd + k*PID_output
  final_speed.final_vx = (1.0f - k) * ramp_vx + k * chassis_vx_e;
  final_speed.final_vy = (1.0f - k) * ramp_vy + k * chassis_vy_e;
  final_speed.final_vw = (1.0f - k) * ramp_vw + k * chassis_vw_e;

  // final_speed.final_vw += g_imu_w_correction;

  Omni_inverse_calc();
  CAN_cmd_chassis(chassis.current_out[0], chassis.current_out[1],
                  chassis.current_out[2], chassis.current_out[3]);
}