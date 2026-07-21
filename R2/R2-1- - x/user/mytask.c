#include "mytask.h"
#include "CAN_receive.h"
#include "FreeRTOS.h"
#include "R2006.h"
#include "arm.h"
#include "contorl_two.h"
#include "control_Cylinder.h"
#include "ecpid.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "unitree.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include "ros_uart.h"
#include "hi14_driver.h"    /* HI14 IMU: 航向锁定 / 急停检测 */
#include "modbus.h"         /* Modbus_Restart: IMU 连续超时恢复 */

static float clampf(float v, float lo, float hi);
static float slew_to(float current, float target, float step);


TaskHandle_t StartTask_Handler;
TaskHandle_t ChassisTask_Handler;
TaskHandle_t IMUTask_Handler;
TaskHandle_t L1Task_Handler;
TaskHandle_t HOWTask_Handler;
TaskHandle_t M8010Task_Handler;
TaskHandle_t ROSTask_Handler;

HOW_TO_CONTROL how = ONE;//母状态，1:ONE,2:TWO,3:THREE
//1区
GRIP_CONTROL grip_run = grip_close;//气抓东西状态
LIFT_CONTROL lift_run = lift_down;// 气抓升降状态
ROTATE_CONTROL rotate_run = rotate_stop;// 气抓旋转状态
//其他
ARM_HOW arm_how = stop;// arm状态
int arm_ros = 0;
Wheel_Speed chassis_cmd_speed = {0};
Order order = open;
ROS_UartFrame ROS2; // 全局变量存储最新的ROS数据帧
uint32_t ros_last_ms = 0;
uint32_t ros_vx_zero_start_ms = 0;

extern Wheel_type wheel;
/* ======== M8010 通信任务 ======== */
extern UART_HandleTypeDef huart6;
extern MotorCmd_t cmd[MOTOR_NUM];
extern MotorData_t data[MOTOR_NUM];
extern uint8_t Temp_buffer[64];
YS_RUN_E y_run = stop_;
void start_task(void *pvParameters) {
  taskENTER_CRITICAL();
  xTaskCreate((TaskFunction_t)How_task, "How_task", (uint16_t)HOW_SIZE, (void *)NULL, (UBaseType_t)HOW_TASK_PRIO, (TaskHandle_t *)&HOWTask_Handler);
  xTaskCreate((TaskFunction_t)chassis_task, "chassis_task", (uint16_t)CHASSIS_STK_SIZE, (void *)NULL, (UBaseType_t)CHASSIS_TASK_PRIO, (TaskHandle_t *)&ChassisTask_Handler);
  xTaskCreate((TaskFunction_t)imu_task, "imu_task", (uint16_t)IMU_STK_SIZE, (void *)NULL, (UBaseType_t)IMU_TASK_PRIO, (TaskHandle_t *)&IMUTask_Handler);
  xTaskCreate((TaskFunction_t)l1_task, "l1_task", (uint16_t)L1_STK_SIZE, (void *)NULL, (UBaseType_t)L1_TASK_PRIO, (TaskHandle_t *)&L1Task_Handler);
//  xTaskCreate((TaskFunction_t)M8010_CommTask, "M8010_task", (uint16_t)M8010_STK_SIZE, (void *)NULL, (UBaseType_t)M8010_TASK_PRIO, (TaskHandle_t *)&M8010Task_Handler);
  xTaskCreate((TaskFunction_t)ros_task, "ros_task", (uint16_t)ROS_SIZE, (void *)NULL, (UBaseType_t)ROS_TASK_PRIO, (TaskHandle_t *)&ROSTask_Handler);
  taskEXIT_CRITICAL();            // 退出临界区
  vTaskDelete(StartTask_Handler); // 删除开始任务
}
int o1,o2,o3;
static void update_odometry(void);
void chassis_task(void *pvParameters) {
  (void)pvParameters;
  Chassis_init();
  for (;;) {
    wheel_calc();
    update_odometry();
	  o1++;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
// IMU 任务
#define DEG2RAD         (PI / 180.0f)  /* 度→弧度 */
#define VMAX_REF        1.0f   // 底盘最大平移速度参考值
#define KP_HEADING_MAX  2.5f  // 静止时Kp
#define KP_HEADING_MIN  2.4f   // 全速时Kp
#define KI_HEADING_MAX  0.02f  // 静止时Ki
#define KI_HEADING_MIN  0.04f  // 全速时Ki
#define KD_HEADING_MAX  0.05f  // 静止时Kd
#define KD_HEADING_MIN  0.02f  // 全速时Kd
#define HEADING_DEADZONE 0.1f  // 航向纠偏死区(deg)
#define CMD_Z_LIMIT     100.0f  // 纠偏角速度上限(deg/s)
#define CMD_Z_RAMP_STEP 1.0f    // cmd_z斜坡步长(deg/s/周期)
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
  float target_yaw = 0.0f;         /**< 目标航向角  */
  uint8_t was_rotating = 0; /**< 上一次是否在旋转  */
  uint8_t heading_locked =0; /**< 航向`首次锁定标志  */
  HI14_SensorData_t imu_data; /**< 本地 IMU 数据副本 */
  /* cmd_z 斜坡状态(跨周期保持): 平滑纠偏输出,消除突变 */
  static float cmd_z_ramp = 0.0f;
  /* 连续超时计数: 超过阈值时主动重启 Modbus DMA 接收 */
  uint16_t consecutive_timeout = 0;
  uint16_t imu_time = 15;
  for (;;) {
    /* ================================================================
     * HI14 IMU 每周期读取 + 航向锁定 (每 2ms 执行一次)
     * ================================================================ */
      if (fabsf(heading_correction) < 0.001f) {heading_correction = 0.0f;}

      taskENTER_CRITICAL();
      float snap_Mvx = chassis_cmd_speed.f_Mvx;
      float snap_Mvy = chassis_cmd_speed.f_Mvy;
      float snap_Mvw = chassis_cmd_speed.f_Mvw;
      taskEXIT_CRITICAL();

      /* 读取传感器数据 */
      if (HI14_ReadAllSensors(&imu_data) == HI14_OK) {
        consecutive_timeout = 0;
        g_imu_data = imu_data; /* 更新全局数据 (供其他模块使用) */
        /* ---- 首次锁定: 上电后第一次成功读取 IMU 时,锁定当前航向为目标 ---- */
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
          /* 2. Kp/Ki/Kd 随速度反比*/
          speed_ratio = fminf(cmd_speed / VMAX_REF, 1.0f);
          heading_Kp_adaptive = KP_HEADING_MAX
                             - (KP_HEADING_MAX - KP_HEADING_MIN) * speed_ratio;
          heading_Ki_adaptive = KI_HEADING_MAX
                             - (KI_HEADING_MAX - KI_HEADING_MIN) * speed_ratio;
          heading_Kd_adaptive = KD_HEADING_MAX
                             - (KD_HEADING_MAX - KD_HEADING_MIN) * speed_ratio;
          /* 3. 计算纠偏*/
          heading_correction = Mecanum_HeadingCorrection(target_yaw, heading_Kp_adaptive, heading_Ki_adaptive, heading_Kd_adaptive);
          /* 4. 死区: 消除IMU噪声引起的微抖 */
          if (fabsf(heading_correction) < HEADING_DEADZONE) {
            heading_correction = 0.0f;
          }
          /* 5. 限幅: 纠偏角速度上限 */
          heading_correction = clampf(heading_correction, -CMD_Z_LIMIT, CMD_Z_LIMIT);
        }
        /* 6. 斜坡*/
        cmd_z_ramp = slew_to(cmd_z_ramp, heading_correction, CMD_Z_RAMP_STEP);
        cmd_z = cmd_z_ramp;
      } else {
        consecutive_timeout++;
        if (consecutive_timeout >= imu_time) {
          Modbus_Restart(&g_modbus_inst);
          consecutive_timeout = 0;
        }
        cmd_z_ramp *= 0.9f;
        cmd_z = cmd_z_ramp;
      }
    {
      float fz = -cmd_z ;
      taskENTER_CRITICAL();
      chassis_cmd_speed.f_z = fz * 0.0174533f;
      taskEXIT_CRITICAL();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
 }
}
/**************************激光任务******************************/
float laser_distance_m = 0.0f;

void l1_task(void *pvParameters)
{
    (void)pvParameters;
    extern int value[2];
    L1_Init();
    for (;;)
    {
        L1_ReadDistance(&laser_distance_m);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
// ****************************ROS-how 任务******************************
uint32_t lift_up_accumulated_ms = 0;
uint32_t lift_up_last_tick = 0;
  void roshow_change(void) {
    uint32_t now = HAL_GetTick();
    ROS_UartFrame snap_ROS2;
    taskENTER_CRITICAL();
    snap_ROS2 = ROS2;
    taskEXIT_CRITICAL();

    if (snap_ROS2.valid && (uint32_t)(HAL_GetTick() - ros_last_ms) <= TIMEOUT_MS) {
      if (snap_ROS2.linear_vel == 0.0f) {
        if (ros_vx_zero_start_ms == 0) ros_vx_zero_start_ms = HAL_GetTick();
      } else {
        ros_vx_zero_start_ms = 0;
      }
    } else {
      ros_vx_zero_start_ms = 0;
    }

    if (snap_ROS2.nav_state ==1) {
      how = ONE;
      switch (snap_ROS2.grip_run) {
        case 0:
        grip_run = grip_close;
          break;
        case 1:
        grip_run = grip_put;
          break;
      }
      if (lift_up_last_tick != 0) {
        lift_up_accumulated_ms += (now - lift_up_last_tick);
        lift_up_last_tick = now;
      }
      switch (snap_ROS2.lift_run) {
        case 0:
        lift_run = lift_down;
          break;
        case 1:
        lift_run = lift_up;
        if (lift_up_last_tick == 0) { // 首次进入上升状态，启动计时
          lift_up_last_tick = now;
        }
          break;
      }
      if (lift_up_accumulated_ms >= 100000U) {
        grip_run = grip_put;
      }
      switch (snap_ROS2.rotate_run) {
        case 0:
        rotate_run = rotate_stop;
          break;
        case 1:
        rotate_run = rotate_turn;
          break;
      }
    } else if (snap_ROS2.nav_state ==2){
      how = TWO;
      arm_ros = snap_ROS2.arm_how;
    } else if (snap_ROS2.nav_state == 3){
      how = THREE;
      arm_ros = snap_ROS2.arm_how;
    }
    return;
}

// ****************************M8010 任务******************************
// void M8010_CommTask(void *pvParameters) {
//  (void)pvParameters;
//  // 确保 UART 外设已使能
//  huart6.Instance->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
//  // 停止 DMA 接收，由本任务手动收发
//  HAL_UART_AbortReceive(&huart6);
//  // 关闭 UART6 中断，防止 HAL IRQ 偷读 DR
//  __HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
//  __HAL_UART_DISABLE_IT(&huart6, UART_IT_PE);
//  __HAL_UART_DISABLE_IT(&huart6, UART_IT_ERR);
//  HAL_NVIC_DisableIRQ(USART6_IRQn);

//  TickType_t xLastWakeTime = xTaskGetTickCount();
//  const TickType_t xPeriod   = pdMS_TO_TICKS(2);
//  USART_TypeDef *const u = huart6.Instance;
//  uint8_t *tx_ptr;
//  uint16_t tx_rem;

//  for (;;) {
//    for (int i = 0; i < 3; i++) {
//      // ① 组帧
//      modify_data(&cmd[i]);
//      tx_ptr = MotorCmd_get_motor_send_data(&cmd[i]);
//      tx_rem = (uint16_t)cmd[i].hex_len;

//      // ② 发送命令，同时丢弃所有回环数据(防 ORE)
//      while (tx_rem > 0) {
//        if (u->SR & USART_SR_TXE) {
//          u->DR = *tx_ptr++;
//          tx_rem--;
//        }
//        if (u->SR & USART_SR_RXNE) { (void)u->DR; }
//        if (u->SR & USART_SR_ORE)  { (void)u->DR; }
//      }

//      // ③ 等待发送完成，继续丢弃回环
//      {
//        uint32_t t0 = HAL_GetTick();
//        while (!(u->SR & USART_SR_TC)) {
//          if (u->SR & USART_SR_RXNE) { (void)u->DR; }
//          if (u->SR & USART_SR_ORE)  { (void)u->DR; }
//          if ((HAL_GetTick() - t0) > 2U) break;
//        }
//      }

//      // ④ 接收电机回复: 按 0xFD 0xEE 帧头同步，收满 16 字节
//      {
//        uint8_t rx_buf[16];
//        uint16_t rx_idx = 0;
//        uint32_t t0 = HAL_GetTick();
//        while ((HAL_GetTick() - t0) < 1U) {
//          if (u->SR & USART_SR_ORE)  { (void)u->DR; continue; }
//          if (!(u->SR & USART_SR_RXNE)) continue;
//          uint8_t byte = (uint8_t)(u->DR);
//          if (rx_idx == 0) {
//            if (byte != 0xFD) continue;
//            rx_buf[0] = byte;
//            rx_idx = 1;
//          } else if (rx_idx == 1) {
//            if (byte != 0xEE) { rx_idx = 0; continue; }
//            rx_buf[1] = byte;
//            rx_idx = 2;
//          } else {
//            rx_buf[rx_idx++] = byte;
//            if (rx_idx >= 16) break;
//          }
//        }

//        // ⑤ 解析数据
//        if (rx_idx >= 16) {
//          Unitree_OnUart6Rx(rx_buf, 16);
//        }
//      }
//      // ⑥ 计算误差
//      MotorData_calculate_error(&data[i], cmd[i].tar_pos, cmd[i].tar_w, cmd[i].tar_t);
//    }
//    vTaskDelayUntil(&xLastWakeTime, xPeriod);
//  }
// }

// YS_RUN_E y_run = stop_;
// YS-IRTM 红外模块处理函数
/**
 * @brief 红外遥控器处理函数
 * @details 根据接收到的红外遥控器键值控制机械臂和抓手的动作
 *          包含超时保护机制，当150ms内无有效接收到数据时执行紧急停车
 */
void YS_IRTM_Process(void)
{
    if ((HAL_GetTick() - YS_IRTM_GetLastRecvTime()) > IR_TIMEOUT_MS)
    {
        y_run = stop_;
        return;
    }

    if (!YS_IRTM_HasFrame())
    {
        return;
    }

    uint8_t key = YS_IRTM_GetKey();

    switch (key)
    {
        case 0x00:
            y_run = stop_;
            break;
        case 0x01:
            y_run = grip_put_;
            break;
        case 0x02:
            y_run = three_x;
            break;
        case 0x03:
            y_run = three_put;
            break;
        case 0xFF:
            break;
        default:
            break;
    }
}
//two:先吸再取；无所谓/ three:先吸再取；先放再松
  void two_task() {
  switch(arm_how) {
   case stop:
  arm_final();
  arm_control_motors();
    break;
    case getx_L:
    case getx_H:
  Cylinder_Controlx();
  arm_final();
  arm_control_motors();
      break;
    case arm_put:
    case get_put:
  Cylinder_Controly();
  Cylinder_Controlx();
  arm_final();
  arm_control_motors();
      break;
}
}

void three_task() {
  switch(arm_how) {
   case stop:
  arm_final();
  arm_control_motors();
    break;
    case get_put:
  // Cylinder_Controlx();
  Cylinder_Controly();
  arm_final();
  arm_control_motors();
    break;
    case arm_put:
  Cylinder_Controly();
  arm_final();
  arm_control_motors();
    break;
    case stop_3:
  Cylinder_Controly();
  arm_final();
  arm_control_motors();
      break;
  }
}

void How_task(void *pvParameters) {
  (void)pvParameters;
  Cylinder_Init();
  R2006_Init();
  Unitree_Init();
  YS_IRTM_Init();
  for (;;) {
    roshow_change();
    YS_IRTM_Process();
    if (y_run == grip_put_) {
      grip_run = grip_put;
    }
    switch (how) {
    case ONE:
     Cylinder_Control();
      R2006_Control();
      break;
    case TWO:
     two_task();
      break;
    case THREE:
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

/* ======== 里程计模块 (500Hz, chassis_task 上下文) ======== */
#define K_ODOM          1.0f
Odom_Data odom = {0};

static void update_odometry(void) {
    static uint8_t  angles_inited = 0;
    static float    last_angle[4] = {0};
    static float    last_omni_angle[2] = {0};
    const float     deadzone = 0.01f;

    float d0 = wheel.wheel_motor[0].real_angle2;
    float d1 = wheel.wheel_motor[1].real_angle2;
    float d2 = wheel.wheel_motor[2].real_angle2;
    float d3 = wheel.wheel_motor[3].real_angle2;
    float o0 = wheel.wheelO_motor[0].real_angle2;
    float o1 = wheel.wheelO_motor[1].real_angle2;

    if (!angles_inited) {
        last_angle[0] = d0; last_angle[1] = d1;
        last_angle[2] = d2; last_angle[3] = d3;
        last_omni_angle[0] = o0; last_omni_angle[1] = o1;
        angles_inited = 1;
        return;
    }

    d0 -= last_angle[0]; d1 -= last_angle[1];
    d2 -= last_angle[2]; d3 -= last_angle[3];
    o0 -= last_omni_angle[0]; o1 -= last_omni_angle[1];

    if (fabsf(d0) < deadzone) d0 = 0.0f;
    if (fabsf(d1) < deadzone) d1 = 0.0f;
    if (fabsf(d2) < deadzone) d2 = 0.0f;
    if (fabsf(d3) < deadzone) d3 = 0.0f;
    if (fabsf(o0) < deadzone) o0 = 0.0f;
    if (fabsf(o1) < deadzone) o1 = 0.0f;

    last_angle[0] = wheel.wheel_motor[0].real_angle2;
    last_angle[1] = wheel.wheel_motor[1].real_angle2;
    last_angle[2] = wheel.wheel_motor[2].real_angle2;
    last_angle[3] = wheel.wheel_motor[3].real_angle2;
    last_omni_angle[0] = wheel.wheelO_motor[0].real_angle2;
    last_omni_angle[1] = wheel.wheelO_motor[1].real_angle2;

    static const float R_calib[4] = {
        M_WHEEL_Rr + 0.0085f,
        M_WHEEL_Rr + 0.0025f,
        M_WHEEL_Rr + 0.0088f,
        M_WHEEL_Rr + 0.0030f
    };
    float s0 = d0 * DEG2RAD * R_calib[0];
    float s1 = d1 * DEG2RAD * R_calib[1];
    float s2 = d2 * DEG2RAD * R_calib[2];
    float s3 = d3 * DEG2RAD * R_calib[3];

    static const float R_omni[2] = { OMIN_WHEEL_Rr, OMIN_WHEEL_Rr + 0.05f };
    float so0 = o0 * DEG2RAD * R_omni[0];
    float so1 = o1 * DEG2RAD * R_omni[1];
    float delta_omni_x = (so0 - so1) * SQRT2 * 0.5f;

    float delta_mecanum_x = ( s0 - s1 + s2 - s3) * 0.25f;
    float delta_mecanum_y = ( s0 + s1 - s2 - s3) * 0.25f;

    float delta_local_x = (delta_mecanum_x + delta_omni_x) * 0.5f;
    float delta_local_y = delta_mecanum_y * 0.5f;

    odom.x += delta_local_x * K_ODOM;
    odom.y += delta_local_y * K_ODOM;

    odom.yaw     = g_imu_data.yaw;
    odom.yaw_rad = g_imu_data.yaw * DEG2RAD;
}

// /* ======== 位置环工具函数 ======== */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float slew_to(float current, float target, float step)
{
  float diff = target - current;
  if (diff > step) return current + step;
  if (diff < -step) return current - step;
  return target;
}

// static void advance_navigation_target(float *target_x, float *target_y,
//                     float dx, float dy)
// {
//   *target_x += dx;
//   *target_y += dy;
// }

// /**
//  * @brief  位置环: 根据里程计误差计算底盘速度指令
//  * @return 1 = 已到达目标点, 0 = 未到达
//  */

//  /* ======== 位置环参数宏 ======== */
// #define POS_KP_X        1.5f    /* X轴P */
// #define POS_KP_Y        1.5f    /* Y轴P */
// #define POS_KI_X        0.0f
// #define POS_KI_Y        0.0f
// #define POS_VX_MAX      0.3f
// #define POS_VY_MAX      0.2f
// #define POS_REACH_X     0.05f
// #define POS_REACH_Y     0.05f
// #define POS_INTEGRAL_MAX 0.3f
// #define POS_DT          0.002f  /* 位置环周期 (2ms, 500Hz) */

// #define POS_KP_W        2.0f    /* 航向P */
// #define POS_KI_W        0.0f    /* 航向I */
// #define POS_KD_W        0.1f    /* 航向D */
// #define POS_W_MAX       1.5f    /* 角速度限幅 rad/s */
// #define POS_REACH_W     0.05f   /* 航向到达阈值 rad */
// #define POS_W_INTEG_MAX 0.5f

// uint8_t position_loop_update(float target_x, float target_y, float target_yaw_rad,
//                               float *out_vx, float *out_vy, float *out_w) {
//     float ox, oy, oyaw;
//     taskENTER_CRITICAL();
//     ox = odom.x;
//     oy = odom.y;
//     oyaw = odom.yaw_rad;
//     taskEXIT_CRITICAL();

//     /* 局部坐标系: 直接误差, 不做世界坐标旋转 */
//     float err_x = target_x - ox;
//     float err_y = target_y - oy;

//     /* 航向误差: 归一化到 [-PI, PI] */
//     float err_w = target_yaw_rad - oyaw;
//     err_w = fmodf(err_w, 2.0f * PI);
//     if (err_w > PI)  err_w -= 2.0f * PI;
//     if (err_w < -PI) err_w += 2.0f * PI;

//     static float integral_w = 0.0f;
//     static float prev_err_w = 0.0f;
//     static float prev_target_yaw = 0.0f;
//     if (target_yaw_rad != prev_target_yaw) {
//         integral_w = 0.0f;
//         prev_err_w = 0.0f;
//         prev_target_yaw = target_yaw_rad;
//     }

//     integral_w += err_w * POS_DT;
//     integral_w = clampf(integral_w, -POS_W_INTEG_MAX, POS_W_INTEG_MAX);
//     float deriv_w = (err_w - prev_err_w) / POS_DT;
//     prev_err_w = err_w;

//     float cmd_vx = POS_KP_X * err_x;
//     float cmd_vy = POS_KP_Y * err_y;
//     float cmd_w  = POS_KP_W * err_w + POS_KI_W * integral_w + POS_KD_W * deriv_w;

//     cmd_vx = clampf(cmd_vx, -POS_VX_MAX, POS_VX_MAX);
//     cmd_vy = clampf(cmd_vy, -POS_VY_MAX, POS_VY_MAX);
//     cmd_w  = clampf(cmd_w,  -POS_W_MAX,  POS_W_MAX);

//     *out_vx = cmd_vx;
//     *out_vy = cmd_vy;
//     *out_w  = cmd_w;

//     uint8_t rx = (fabsf(err_x) < POS_REACH_X) ? 1 : 0;
//     uint8_t ry = (fabsf(err_y) < POS_REACH_Y) ? 2 : 0;
//     uint8_t rw = (fabsf(err_w) < POS_REACH_W) ? 4 : 0;
//     return rx | ry | rw;
// }

// #define STARTUP_DURATION_MS  1000U   /* 状态切换后限幅保护时长(ms) */
// #define STARTUP_VXY_HIGH     0.8f    /* 限幅前半段平移速度上限(m/s) */
// #define STARTUP_VXY_LOW      0.5f    /* 限幅后半段平移速度上限(m/s) */
// #define STARTUP_W_HIGH       0.8f    /* 限幅前半段角速度上限(rad/s) */
// #define STARTUP_W_LOW        0.5f    /* 限幅后半段角速度上限(rad/s) */
// #define NAV_SMOOTH_ALPHA     0.8f   /* 速度低通滤波系数(0~1,越小越平滑) */
//   float cmd_vx = 0.0f;
//   float cmd_vy = 0.0f;
//   float cmd_w = 0.0f;
//   float target_x = 0.0f, target_y = 0.0f;
//   float nav_target_yaw = 0.0f;
//   uint8_t nav_yaw_locked = 0;
//   Order prev_order = open;
//   uint32_t order_change_ms = 0;
//   static float smooth_vx = 0.0f;
//   static float smooth_vy = 0.0f;
//   static float smooth_w = 0.0f;
//   extern int value[2];
//   int change = 0;
//   uint32_t fifth_entry_ms = 0; /* fifth状态进入时刻, 用于非阻塞延时 */
void ros_task(void *pvParameters) {
    (void)pvParameters;
    ROS_UartInit();
    ROS_UartFrame received_frame;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_w = 0.0f;

    for (;;) {
        // sbus_loop();
        // 添加数据获取逻辑
        if(ROS_GetFrame(&received_frame)) {
          taskENTER_CRITICAL();
          ROS2 = received_frame;  // 临界区内更新全局数据，防止撕裂读
          taskEXIT_CRITICAL();
          ros_last_ms = HAL_GetTick();
        }
//		ROS_ProcessBuffer(uint8_t* buf);
        // SBUS_Handle();
        ROS_DataUnpack();
        if (ROS2.valid && (uint32_t)(HAL_GetTick() - ros_last_ms) <= TIMEOUT_MS) {
          cmd_vx = ROS2.linear_vel;
          cmd_vy = ROS2.lateral_vel;
          cmd_w = ROS2.angular_vel;
        } else {
          cmd_vx = 0.0f;
          cmd_vy = 0.0f;
          cmd_w = 0.0f;
        }
        
        chassis_cmd_speed.f_Mvx = cmd_vx;
        chassis_cmd_speed.f_Mvy = cmd_vy;
        /* 航向修正 (deg/s) 换算为 rad/s 后叠加到旋转通道 f_Mvw */
        chassis_cmd_speed.f_Mvw = cmd_w  ;
        chassis_cmd_speed.f_Ovx = cmd_vx;
        chassis_cmd_speed.f_Ovw = cmd_w;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}