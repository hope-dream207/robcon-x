#include "Lift_sucker.h"
#include "control_Cylinder.h"
#include "CAN_receive.h"
#include "ecpid.h"
#include "mytask.h"
#include "sbus.h"
#include <math.h>

#define LIFT_STEP_MAX      15.0f
#define SUCKER_STEP_MAX    10.0f
#define L_SYNC_MAX         50.0f

/* ===== 同步控制参数 ===== */
#define K_SYNC              0.3f    // 交叉耦合同步增益
#define SYNC_ERR_DEADBAND   1.0f    // 同步误差死区(°)
#define USE_SYNC_CALIBRATION  1     // 1=启用同步偏置校准, 0=禁用

/* ===== 软限位参数 ===== */
#define LIFT_ANGLE_MIN       (-975.0f)
#define LIFT_ANGLE_MAX       (15.0f)
#define LIFT_LIMIT_SOFTEN    (50.0f)
#define SUCKER_ANGLE_MIN     (-1900.0f)//2000,-1900
#define SUCKER_ANGLE_MAX     (2000.0f)

/* ===== 速度前馈参数 ===== */
#define MAX_LIFT_RPM         (5000.0f) // 升降电机最大转速(rpm)
#define K_FF_LIFT            (0.35f)   // 速度前馈系数

/* ===== 吸盘精度控制参数 ===== */
#define SUCKER_PREC_I_ERR    (3.0f)    // 精度区积分衰减: 位置误差阈值(°)
#define SUCKER_PREC_I_ATTEN  (0.5f)    // 精度区积分衰减系数

Lift_su_type Lift_su;
extern ;
extern Lift_su_Speed lift_su_speed;

static float lift_delta = 0.0f;
static float sucker_target = 0.0f;
static float m0_lock = 0.0f;
static float m1_lock = 0.0f;
static uint8_t inited = 0;
static uint8_t was_active = 0;
static float sync_offset = 0.0f;
static uint8_t calib_done = 0;
static float sucker_delta = 0.0f;
float target_sucker = 0.0f;
float target_lift = 0.0f;
static uint8_t sucker_i_att = 0;

static float clampf(float x, float min_v, float max_v) {
  return fmaxf(fminf(x, max_v), min_v);
}

static float apply_deadband(float x, float deadband) {
  if (fabsf(x) < deadband) {
    return 0.0f;
  }
  return x;
}

void Lift_su_Reset(void) {
  for (int i = 0; i < 2; i++) {
    PID_clear(&Lift_su.lift_pid[i]);
    ECPID_clear(&Lift_su.lift_ecpid[i]);
  }
  PID_clear(&Lift_su.sucker_pid);
  ECPID_clear(&Lift_su.sucker_ecpid);

  lift_delta = 0.0f;
  sucker_delta = 0.0f;
  sucker_target = (float)Lift_su.lift_su_motor_measure[2].real_angle1;
  m0_lock = (float)Lift_su.lift_su_motor_measure[0].real_angle2;
  m1_lock = (float)Lift_su.lift_su_motor_measure[1].real_angle2;
  inited = 0;
  was_active = 0;
  sync_offset = 0.0f;
  calib_done = 0;
  sucker_i_att = 0;

  Lift_su.L_a[0] = (float)Lift_su.lift_su_motor_measure[0].real_angle2;
  Lift_su.L_a[1] = (float)Lift_su.lift_su_motor_measure[1].real_angle2;
  Lift_su.su_a = sucker_target;
  Lift_su.L_rpm_Mset[0] = 0.0f;
  Lift_su.L_rpm_Mset[1] = 0.0f;
  Lift_su.su_rpm_Mset = 0.0f;
  Lift_su.current_outL[0] = 0;
  Lift_su.current_outL[1] = 0;
  Lift_su.current_outsu = 0;
}
void Lift_su_Init(void) {
  // 电机0 PID参数（可自定义不同值）
  Lift_su.ECPID_LIFT[0][0] = 25.0f; // 电机0 P
  Lift_su.ECPID_LIFT[0][1] = 0.1f;  // 电机0 I
  Lift_su.ECPID_LIFT[0][2] = 0.6f;  // 电机0 D
  Lift_su.PID_LIFT[0][0] = 11.0f;   // 电机0 P
  Lift_su.PID_LIFT[0][1] = 0.5f;    // 电机0 I
  Lift_su.PID_LIFT[0][2] = 0.5f;    // 电机0 D

  Lift_su.ECPID_LIFT[1][0] = 25.0f; 
  Lift_su.ECPID_LIFT[1][1] = 0.1f;  
  Lift_su.ECPID_LIFT[1][2] = 0.6f;  
  Lift_su.PID_LIFT[1][0] = 11.0f;
  Lift_su.PID_LIFT[1][1] = 0.5f;
  Lift_su.PID_LIFT[1][2] = 0.5f;

  // 电机2 PID参数（可自定义不同值），吸盘
  Lift_su.ECPID_sucker[0] = 13.0f;
  Lift_su.ECPID_sucker[1] = 0.05f;
  Lift_su.ECPID_sucker[2] = 1.0f;

  Lift_su.PID_sucker[0] = 10.0f;
  Lift_su.PID_sucker[1] = 0.1f;
  Lift_su.PID_sucker[2] = 0.3f;
  // Lift 电机PID 结构体初始化
  for (int i = 0; i < 2; i++) {
    ECPID_init(&Lift_su.lift_ecpid[i], ECPID_POSITION, Lift_su.ECPID_LIFT[i],
               6200.0f, 1000.0f, 0.2f, 5.0f, 0.5f, 0.2f);
    PID_init(&Lift_su.lift_pid[i], PID_POSITION, Lift_su.PID_LIFT[i], 12000.0f,
             2000.0f, 6.0f, 100.0f, 0.5f, 0.3f);
  }
  ECPID_init(&Lift_su.sucker_ecpid, ECPID_POSITION, Lift_su.ECPID_sucker, 5000,
             400.0f, 0.2f, 30.0f, 0.5f, 0.1f);
  PID_init(&Lift_su.sucker_pid, PID_POSITION, Lift_su.PID_sucker, 8000.0f,
           500.0f, 80.0f, 50.0f, 0.5f, 0.2f);

  CAN_cmd_Lift_su_motor(0, 0, 0); // 控制电机0和电机1
}

void Lift_su_Control(uint8_t is_active) {
  //*****************算法与设定*****************//
  // 交叉耦合同步 + 速度前馈 + 软限位
  // 同步策略：两电机跟踪同一公共目标 common_target，同步误差双向分配到两电机。
  // 速度前馈：摇杆直接映射 RPM 叠加到位置环输出，提高响应。
  // 软限位：接近机械限位时在 RPM 层衰减，防止位置环/速度环积分饱和。
  //  超出限位后反向推回。
  float add_L = 0;
  float add_S = 0;
  float sucker_angle = Lift_su.lift_su_motor_measure[2].real_angle1;

  if (!inited) {
    m0_lock = (float)Lift_su.lift_su_motor_measure[0].real_angle2;
    m1_lock = (float)Lift_su.lift_su_motor_measure[1].real_angle2;
    lift_delta = 0.0f;
    sucker_target = (float)Lift_su.lift_su_motor_measure[2].real_angle1;
    sucker_delta = 0.0f;
    sync_offset = (float)Lift_su.lift_su_motor_measure[0].real_angle2+ (float)Lift_su.lift_su_motor_measure[1].real_angle2;
    calib_done = 1;
    inited = 1;
  }
  // 根据吸盘角度位置调整速度增益（距离衰减）
  // 三个精度区：中心×0.02(精调)，向边界线性恢复到×1.0(全速)
  float sucker_factor = 1.0f;
  uint8_t in_precision_zone = 0;
  // 精度区1: [200, 170], 中心190, 半宽10°
  if (sucker_angle >= 180.0f && sucker_angle <= 200.0f) {
    float dist = fabsf(sucker_angle - 190.0f) / 10.0f;
    sucker_factor = 0.2f + 0.5f * dist;
    in_precision_zone = 1;
  }
  // 精度区3: [-6, 6], 中心0, 半宽15°
  else if (sucker_angle >= -15.0f && sucker_angle <= 15.0f) {
    float dist = fabsf(sucker_angle - 0.0f) / 6.0f;
    sucker_factor = 0.2f + 0.5f * dist;
    in_precision_zone = 1;
  }
  
  // 精度区3: [570, 590], 中心580, 半宽10°
  else if (sucker_angle >= 570.0f && sucker_angle <= 590.0f) {
    float dist = fabsf(sucker_angle - 580.0f) / 10.0f;
    sucker_factor = 0.2f + 0.5f * dist;
    in_precision_zone = 1;
  }
  // // 精度区4: [170, 210], 中心190, 半宽20°
  // else if (sucker_angle >= 170.0f && sucker_angle <= 210.0f) {
  //   float dist = fabsf(sucker_angle - 190.0f) / 20.0f;
  //   sucker_factor = 0.02f + 0.98f * dist;
  //   in_precision_zone = 1;
  // }
  // // 精度区5: [-580, -550], 中心-565, 半宽15°
  // else if (sucker_angle >= -580.0f && sucker_angle <= -550.0f) {
  //   float dist = fabsf(sucker_angle - (-565.0f)) / 15.0f;
  //   sucker_factor = 0.02f + 0.98f * dist;
  //   in_precision_zone = 1;
  // }
  
 // 摇杆输入限幅,步长限制
  add_L = clampf(lift_su_speed.f_Lfa, -LIFT_STEP_MAX, LIFT_STEP_MAX);
  add_S = clampf(lift_su_speed.f_sufa, -SUCKER_STEP_MAX, SUCKER_STEP_MAX) * sucker_factor;
  if (is_active) {
    lift_delta += add_L;
    sucker_delta += add_S;
    target_sucker = sucker_target + sucker_delta;
    if (target_sucker < SUCKER_ANGLE_MIN) {
      sucker_delta = SUCKER_ANGLE_MIN - sucker_target;
    } else if (target_sucker > SUCKER_ANGLE_MAX) {
      sucker_delta = SUCKER_ANGLE_MAX - sucker_target;
    }

    target_lift = m0_lock - lift_delta;
    if (target_lift < LIFT_ANGLE_MIN) {
      lift_delta = m0_lock - LIFT_ANGLE_MIN;
    } else if (target_lift > LIFT_ANGLE_MAX) {
      lift_delta = m0_lock - LIFT_ANGLE_MAX;
    }
  } else if (was_active) {
    Lift_su_Reset();
  }
  was_active = is_active;

  // 两电机对置安装: motor0角度↓=上升, motor1角度↑=上升
  // 共同目标: common_target = m0_lock - lift_delta
  // 同步误差: sync_err = current0 + current1 (理想值=0，对置镜像)
  // 同步误差分配到两电机，共同收敛
  float current0 = (float)Lift_su.lift_su_motor_measure[0].real_angle2;
  float current1 = (float)Lift_su.lift_su_motor_measure[1].real_angle2;

 // 首次激活时记录零位
#if USE_SYNC_CALIBRATION
  if (!calib_done && is_active) {
    sync_offset = current0 + current1;
    calib_done = 1;
  }
#endif

  float common_target = m0_lock - lift_delta;

  float target0, target1;
  if (is_active) {
    float sync_err = current0 + current1 - sync_offset;
    sync_err = apply_deadband(sync_err, SYNC_ERR_DEADBAND);

    float sync_corr = K_SYNC * sync_err;
    sync_corr = clampf(sync_corr, -L_SYNC_MAX, L_SYNC_MAX);

    target0 = common_target - sync_corr;
    target1 = -common_target - sync_corr;
  } else {
    target0 = m0_lock;
    target1 = m1_lock;
  }
 // 吸盘快速定位
  Lift_su.L_a[0] = target0;
  Lift_su.L_a[1] = target1;

  // 吸盘电机目标（单电机，无同步问题）
  Lift_su.su_a = sucker_target + sucker_delta;

  //*****************计算*****************//
  // ===== 速度前馈 =====
  // 摇杆指令直接映射为转速前馈，提高响应
  // 电机0(角度↓=上升): -ff_rpm, 电机1(角度↑=上升): +ff_rpm(对置)
  float ff_rpm = is_active ? (lift_su_speed.f_Lfa * MAX_LIFT_RPM * K_FF_LIFT) : 0.0f;

  // ===== 软限位衰减系数(在RPM层生效，防止位置环/速度环积分饱和) =====
  // 基于电机0角度(主参考)，接近限位时线性衰减RPM，超出后反向推回
  float limit_scale = 1.0f;
  {
    float angle_limit = (float)Lift_su.lift_su_motor_measure[0].real_angle2;

    if (angle_limit < LIFT_ANGLE_MIN + LIFT_LIMIT_SOFTEN) {
      if (angle_limit <= LIFT_ANGLE_MIN) {
        limit_scale = -0.5f;  // 超出下限，反向推回
      } else {
        limit_scale = (angle_limit - LIFT_ANGLE_MIN) / LIFT_LIMIT_SOFTEN;
      }
    } else if (angle_limit > LIFT_ANGLE_MAX - LIFT_LIMIT_SOFTEN) {
      if (angle_limit >= LIFT_ANGLE_MAX) {
        limit_scale = -0.5f;  // 超出上限，反向推回
      } else {
        limit_scale = (LIFT_ANGLE_MAX - angle_limit) / LIFT_LIMIT_SOFTEN;
      }
    }

    if (limit_scale < -0.5f) limit_scale = -0.5f;
  }

  for (int i = 0; i < 2; i++) {
    Lift_su.L_rpm_Mset[i] = ECPID_Calc(
        &Lift_su.lift_ecpid[i],
        (float)Lift_su.lift_su_motor_measure[i].real_angle2, Lift_su.L_a[i]);
    // 速度前馈叠加 (对置安装方向相反)
    Lift_su.L_rpm_Mset[i] += (i == 0) ? (-ff_rpm) : ff_rpm;
    // 软限位衰减 RPM 层(在速度环之前)
    Lift_su.L_rpm_Mset[i] *= limit_scale;
    Lift_su.current_outL[i] = PID_calc(
        &Lift_su.lift_pid[i], (float)Lift_su.lift_su_motor_measure[i].speed_rpm,
        Lift_su.L_rpm_Mset[i]);
  }
  Lift_su.su_rpm_Mset = ECPID_Calc(
      &Lift_su.sucker_ecpid, (float)Lift_su.lift_su_motor_measure[2].real_angle1,
      Lift_su.su_a);
  Lift_su.current_outsu = PID_calc(
      &Lift_su.sucker_pid, (float)Lift_su.lift_su_motor_measure[2].speed_rpm,
      Lift_su.su_rpm_Mset);


  // 离开阈值区域后重置标志
  if (in_precision_zone) {
    float sucker_pos_err = Lift_su.su_a -(float)Lift_su.lift_su_motor_measure[2].real_angle1;//设定减去反馈值
    if (fabsf(sucker_pos_err) < SUCKER_PREC_I_ERR) {
      float sucker_speed = (float)Lift_su.lift_su_motor_measure[2].speed_rpm;
      if ((sucker_pos_err > 0.0f && sucker_speed > 0.0f) ||(sucker_pos_err < 0.0f && sucker_speed < 0.0f)) {
        if (!sucker_i_att) {
          Lift_su.sucker_pid.Iout *= SUCKER_PREC_I_ATTEN;
          sucker_i_att = 1;
        }
      } else {
        sucker_i_att = 0;  // 速度方向反了(已过头)，重置标志
      }
    } else {
      sucker_i_att = 0;    // 离开阈值区域，重置标志
    }
  } else {
    sucker_i_att = 0;      // 离开精度区，重置标志
  }

  //*****************输出*****************//
  CAN_cmd_Lift_su_motor(Lift_su.current_outL[0], Lift_su.current_outL[1], Lift_su.current_outsu);
}