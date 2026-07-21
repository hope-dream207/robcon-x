#include "mytask.h"
#include "CAN_receive.h"
#include "FreeRTOSConfig.h"
#include "Lift_sucker.h"
#include "Omin.h"
#include "control_Cylinder.h"
#include "grip_turnd.h"
#include "queue.h"
#include "stm32f4xx_hal_can.h"
#include <math.h>
#include <stdio.h>

/**气缸能保持
底盘:x1;y0,w2
4<200时:升气缸7，张气缸8,9(转盘6).对于5：上全底盘，下旋转3，给6020
4>200时:伸出气缸6，真空泵7.对于5：上全底盘，下旋转3，给LIFT;左右无,给吸盘2006
4放在HOW,父状态;5放在yaokong,子状态
所以说，底盘是全状态的可以(旋转部分可以在遥控器里面调)
**/
#define SBUS_CENTER_VALUE 992.0f
#define SBUS_RANGE_VALUE 880.0f
#define SBUS_DEADZONE_VALUE 0.06f
#define SBUS_DEADZONE_SCALE (1.0f - SBUS_DEADZONE_VALUE) // 0.94f, 预计算避免运行时减法
#define RC_EXPO_VALUE 0.25f
#define RC_EXPO_C1 (1.0f - RC_EXPO_VALUE)                
#define RC_LPF_ALPHA 0.50f
#define SBUS_TIMEOUT_MS 50.0f
#define CHASSIS_MAX_VX 10.0f
#define CHASSIS_MAX_VY 8.0f
#define CHASSIS_MAX_W 10.0f
#define CHASSIS_MAX_Z 5.0f
int a,b,c;
// 遥控进阶
static float clampf(float x, float min_v, float max_v) {
  return fmaxf(fminf(x, max_v), min_v);
}

static float expo_curve(float x) {
  return RC_EXPO_C1 * x + RC_EXPO_VALUE * x * x * x;
}

static float normalize_sbus_channel(int16_t raw) {
  float x = ((float)raw - SBUS_CENTER_VALUE) / SBUS_RANGE_VALUE;
  x = clampf(x, -1.0f, 1.0f);

  if (fabsf(x) < SBUS_DEADZONE_VALUE) {
    return 0.0f;
  }

  x = (x > 0.0f) ? (x - SBUS_DEADZONE_VALUE) / SBUS_DEADZONE_SCALE: (x + SBUS_DEADZONE_VALUE) / SBUS_DEADZONE_SCALE;
  return expo_curve(x);
}

static float low_pass(float last, float input, float alpha) {
  return last + alpha * (input - last);
}

// 一步完成低通滤波+死区截断，消除尾部漂移
static void apply_cmd(float *cmd, float input, float max_val, float deadzone) {
  float val = low_pass(*cmd, input * max_val, RC_LPF_ALPHA);
  *cmd = (fabsf(val) < deadzone) ? 0.0f : val;
}

TaskHandle_t StartTask_Handler;
TaskHandle_t MOTOR_CHASSISTask_Handler;
TaskHandle_t YAOKONGTask_Handler;
TaskHandle_t GTSTask_Handler;
TaskHandle_t HOWTask_Handler;
TaskHandle_t IMUTask_Handler;
QueueHandle_t uartTxQueue;
QueueHandle_t xCylinderQueue;

volatile float g_imu_w_correction = 0.0f;
static float s_heading_target = 0.0f;
static uint8_t s_heading_locked = 0;

HOW_TO_CONTROL how = CHASSIS;
C_AND_GTLS c_and_gtls = CHASSIS_ALL;
Chassis_Speed f_speed;
Lift_su_Speed lift_su_speed;
GT_Djl6020 GT_g;
YS_RUN_E ys_run = stop;
extern int16_t g_sbus_channels[18];

void start_task(void *pvParameters) {
  taskENTER_CRITICAL();

  /* ------------------------------------------------------------- */

  // 创建底盘任务
  xTaskCreate((TaskFunction_t)motor_CHASSIS_task,
              (const char *)"motor_CHASSIS_task",
              (uint16_t)MOTOR_CHASSIS_SIZE,
              (void *)NULL,
              (UBaseType_t)MOTOR_CHASSIS_TASK_PRIO,
              (TaskHandle_t *)&MOTOR_CHASSISTask_Handler);
  // 创建How任务
  xTaskCreate((TaskFunction_t)How_task,
              (const char *)"How_task",
              (uint16_t)HOW_SIZE,
              (void *)NULL,
              (UBaseType_t)HOW_TASK_PRIO,
              (TaskHandle_t *)&HOWTask_Handler);
  // 创建YAOKONG任务
  xTaskCreate((TaskFunction_t)yaokong_task,
              (const char *)"yaokong_task",
              (uint16_t)YAOKONG_SIZE,
              (void *)NULL,
              (UBaseType_t)YAOKONG_TASK_PRIO,
              (TaskHandle_t *)&YAOKONGTask_Handler);
  // 创建IMU任务
  xTaskCreate((TaskFunction_t)imu_task,
              (const char *)"imu_task",
              (uint16_t)IMU_STK_SIZE,
              (void *)NULL,
              (UBaseType_t)IMU_TASK_PRIO,
              (TaskHandle_t *)&IMUTask_Handler);

  taskEXIT_CRITICAL();
  vTaskDelete(StartTask_Handler);
}

    void C_AND_GTLS_change(void){
    c_and_gtls = (g_sbus_channels[5] > 200) ? GTLS_MOVE : CHASSIS_ALL;}
    void How_change(){
    if(g_sbus_channels[4] > 20 && g_sbus_channels[4] < 800)
    {
        how = CHASSIS;
    }
    // else if(g_sbus_channels[4] > 800 && g_sbus_channels[4] < 1200)
    // {
    //     how = order;
    // }
    else
    {
        how = GET;
    }
  }
void motor_CHASSIS_task(void *pvParameters) {
  (void)pvParameters;
  Omni_Init();
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2);
  for (;;) {
    Omni_calc();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
	  a++;
  }
}
// 红外函数
void YS_SendCmd(void)
{
    switch (ys_run)
    {
    case stop:
        YS_IRTM_Send(0x00);
        break;
    case grip_put:
        YS_IRTM_Send(0x01);
        break;
    case three_x:
        YS_IRTM_Send(0x02);
         break;
    case three_put:
        YS_IRTM_Send(0x03);
        break;
    default:
        break;
    }
}
void How_task(void *pvParameters) {
  (void)pvParameters;
  Cylinder_Init();
  GT_Init();
  Lift_su_Init();
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static TickType_t last_ys_send = 0;
  for (;;) {
    if(ys_run != stop && xTaskGetTickCount() - last_ys_send >= pdMS_TO_TICKS(100))
    {
      YS_SendCmd();
      last_ys_send = xTaskGetTickCount();
    }
    switch (how) {
    case CHASSIS:
      Cylinder_Control();
      if(g_sbus_channels[6] > 200)
      {
        ys_run = grip_put;
      }
      else
      {
        ys_run = stop;
      }
      break;    
    case GET:
      Cylinder_Controlx();
      if(g_sbus_channels[8] > 200)
      {
        ys_run = three_x;
      }
      else if(g_sbus_channels[9] > 200)
      {
        ys_run = three_put;
      }
      else
      {
        ys_run = stop;
      }
      break;
    }
    GT_Control(how == CHASSIS);
    Lift_su_Control(how == GET);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
// 遥控器任务
void yaokong_task(void *pvParameters) {
  (void)pvParameters;
  c_and_gtls = CHASSIS_ALL;
  static HOW_TO_CONTROL last_how = CHASSIS;
  float cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_w = 0.0f;
  float cmd_g  = 0.0f, cmd_l  = 0.0f, cmd_s = 0.0f, cmd_o = 0.0f;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2);

  for (;;) {
    sbus_loop();
    How_change();
    C_AND_GTLS_change();
    if (how != last_how) {
      Cylinder_Init();
      Lift_su_Reset();
      last_how = how;
    }
    float n_ch0 = normalize_sbus_channel(g_sbus_channels[0]);
    float n_ch1 = normalize_sbus_channel(g_sbus_channels[1]);
    float n_ch2 = normalize_sbus_channel(g_sbus_channels[2]);
    float n_ch3 = normalize_sbus_channel(g_sbus_channels[3]);

    apply_cmd(&cmd_vx, n_ch1, CHASSIS_MAX_VX, 0.05f);
    apply_cmd(&cmd_vy, n_ch0, CHASSIS_MAX_VY, 0.05f);
    apply_cmd(&cmd_w,  -1.0f * n_ch3, CHASSIS_MAX_W,  0.05f);
    apply_cmd(&cmd_g,  n_ch2, CHASSIS_MAX_W,  0.05f);
    apply_cmd(&cmd_l,  n_ch2, CHASSIS_MAX_Z,  0.4f);
    apply_cmd(&cmd_s,  n_ch3, CHASSIS_MAX_Z,  0.2f);

    taskENTER_CRITICAL();
    // 默认清零辅助输出，各分支只设置非零项
    lift_su_speed.f_Lfa  = 0.0f;
    lift_su_speed.f_sufa = 0.0f;
    GT_g.f_Gfa           = 0.0f;

    switch (how) {
    case CHASSIS:
      if (c_and_gtls == CHASSIS_ALL) {
        f_speed.f_vx = cmd_vx * 0.35f;
        f_speed.f_vy = cmd_vy * 0.35f;
        f_speed.f_vw = cmd_w * 0.2f;
      } else { // GTLS_MOVE
        f_speed.f_vx = cmd_vx * 0.032f;
        f_speed.f_vy = cmd_vy * 0.025f;
        f_speed.f_vw = cmd_w * 0.02f;
        GT_g.f_Gfa   = cmd_g * 0.015f;
      }
      break;
    case GET:
      if (c_and_gtls == CHASSIS_ALL) {
        f_speed.f_vx = cmd_vx * 0.35f;
        f_speed.f_vy = cmd_vy * 0.35f;
        f_speed.f_vw = cmd_w * 0.2f; // GET状态旋转稍微降点速
        lift_su_speed.f_Lfa  = cmd_l * 0.2f;
      } else { // GTLS_MOVE
        f_speed.f_vx= cmd_vx * 0.032f;
        f_speed.f_vy= cmd_vy * 0.025f;
        f_speed.f_vw=  0.0f;
        lift_su_speed.f_Lfa  = cmd_l * 0.25f;
        lift_su_speed.f_sufa = cmd_s * (-0.1f);
      }
      break;
    }
	b++;
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// IMU 任务
#define VMAX_REF        2.0f   // 底盘最大平移速度参考值
#define KP_HEADING_MAX  1.8f  // 静止时Kp
#define KP_HEADING_MIN  1.5f   // 全速时Kp
#define KI_HEADING_MAX  0.02f  // 静止时Ki
#define KI_HEADING_MIN  0.03f  // 全速时Ki
#define KD_HEADING_MAX  0.05f  // 静止时Kd
#define KD_HEADING_MIN  0.02f  // 全速时Kd
#define HEADING_DEADZONE 1.0f  // 航向纠偏死区(deg)
#define CMD_Z_LIMIT     200.0f  // 纠偏角速度上限(deg/s)
#define CMD_Z_RAMP_STEP 1.0f    // cmd_z斜坡步长(deg/s/周期):
void imu_task(void *pvParameters) {
  (void)pvParameters;
  float cmd_z = 0.0f;
  float cmd_speed = 0.0f;
  float speed_ratio = 0.0f;
  float heading_Kp_adaptive = KP_HEADING_MAX;
  float heading_Ki_adaptive = KI_HEADING_MAX;
  float heading_Kd_adaptive = KD_HEADING_MAX;

  /* ---- HI14 IMU 周期读取变量 ---- */
  float heading_correction = 0.0f; /**< 航向修正角速度 (deg/s) */
  float target_yaw = 0.0f;         /**< 目标航向角 (锁定方向) */
  uint8_t was_rotating = 0; /**< 上一次是否在旋转 (用于旋转停止时锁定新方向) */
  uint8_t heading_locked =0; /**< 航向`首次锁定标志 (上电后首次读IMU时锁定当前方向) */
  HI14_SensorData_t imu_data; /**< 本地 IMU 数据副本 */
  /* cmd_z 斜坡状态(跨周期保持): 平滑纠偏输出,消除突变 */
  static float cmd_z_ramp = 0.0f;
  /* 连续超时计数: 超过阈值时主动重启 Modbus DMA 接收 */
  uint16_t consecutive_timeout = 0;
  uint16_t imu_time = 50;
  for (;;) {
    /* ================================================================
     * HI14 IMU 每周期读取 + 航向锁定 (每 2ms 执行一次)
     * ================================================================ */
      if (fabsf(heading_correction) < 0.001f) {heading_correction = 0.0f;}

      taskENTER_CRITICAL();
      float snap_Mvx = f_speed.f_vx;
      float snap_Mvy = f_speed.f_vy;
      float snap_Mvw = f_speed.f_vw;
      taskEXIT_CRITICAL();

      /* 读取传感器数据 */
      if (HI14_ReadAllSensors(&imu_data) == HI14_OK) {
        consecutive_timeout = 0;
        g_imu_data = imu_data; /* 更新全局数据 (供其他模块使用) */
        /*  上电后第一次成功读取 IMU 时, 锁定当前方向为目标航向 */
        if (!heading_locked) {
          target_yaw = HI14_GetYaw();
          heading_locked = 1;
        }

        if (fabsf(snap_Mvw) >= ROTATE_DEADZONE) {
          target_yaw = HI14_GetYaw();
          heading_correction = 0.0f;
          was_rotating = 1;
        } else {

          if (was_rotating) {
            /* 旋转刚停止: 锁定当前方向为新目标 */
            target_yaw = HI14_GetYaw();
            was_rotating = 0;
          }
          /* === 速度自适应航向纠偏 (PID) === */
          /* 1. 指令平移速度大小 */
          cmd_speed = sqrtf(snap_Mvx * snap_Mvx + snap_Mvy * snap_Mvy);
          /* 2. Kp/Ki/Kd 随速度反比 */
          speed_ratio = cmd_speed / VMAX_REF;
          if (speed_ratio > 1.0f) speed_ratio = 1.0f;
          heading_Kp_adaptive = KP_HEADING_MAX
                             - (KP_HEADING_MAX - KP_HEADING_MIN) * speed_ratio;
          heading_Ki_adaptive = KI_HEADING_MAX
                             - (KI_HEADING_MAX - KI_HEADING_MIN) * speed_ratio;
          heading_Kd_adaptive = KD_HEADING_MAX
                             - (KD_HEADING_MAX - KD_HEADING_MIN) * speed_ratio;
          /* 3. 计算纠偏 */
          heading_correction = Mecanum_HeadingCorrection(target_yaw, heading_Kp_adaptive, heading_Ki_adaptive, heading_Kd_adaptive);
          /* 4. 死区 */
          if (fabsf(heading_correction) < HEADING_DEADZONE) {
            heading_correction = 0.0f;
          }
          /* 5. 限幅 */
          if (heading_correction >  CMD_Z_LIMIT) heading_correction =  CMD_Z_LIMIT;
          if (heading_correction < -CMD_Z_LIMIT) heading_correction = -CMD_Z_LIMIT;
        }
        /* 统一在分支外更新 cmd_z: 旋转时 heading_correction=0 → cmd_z=0 */
        /* 6. 斜坡 */
        {
          float diff = heading_correction - cmd_z_ramp;
          if (diff >  CMD_Z_RAMP_STEP) cmd_z_ramp += CMD_Z_RAMP_STEP;
          else if (diff < -CMD_Z_RAMP_STEP) cmd_z_ramp -= CMD_Z_RAMP_STEP;
          else cmd_z_ramp = heading_correction;
        }
        cmd_z = cmd_z_ramp;
      } else {
        consecutive_timeout++;
        if (consecutive_timeout >= imu_time) {
          Modbus_Restart(&g_modbus_inst);
          consecutive_timeout = 0;
        }
        cmd_z_ramp *= 0.8f;
        cmd_z = cmd_z_ramp;
      }
    {
      float fz = -cmd_z * 0.0174533f;
      taskENTER_CRITICAL();
      f_speed.f_z = fz;
      taskEXIT_CRITICAL();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
 }
}