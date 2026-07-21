/**
  * @file    hi14_driver.h
  * @brief   HiPNUC HI14 系列 IMU 高层驱动 (Modbus RTU)
  *
  * @details
  *
  *   硬件:
  *     - 通信接口: USART3 (PB10-TX, PB11-RX) @ 115200, 8N1
  *     - RS-485 模块: 硬件自动流向控制 (无 RE/DE GPIO)
  *     - HI14 从机地址: 0x50 (出厂默认)
  *
  */

#ifndef HI14_DRIVER_H
#define HI14_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * HI14 寄存器地址定义 (Modbus 保持寄存器)
 * ========================================================================== */

/** 传感器数据寄存器 (起始地址 0x34, 共 16 个寄存器, 连续) */
#define HI14_REG_ACC_X          0x34  /**< 加速度 X 轴 (int16, 大端) */
#define HI14_REG_ACC_Y          0x35  /**< 加速度 Y 轴 (int16, 大端) */
#define HI14_REG_ACC_Z          0x36  /**< 加速度 Z 轴 (int16, 大端) */
#define HI14_REG_GYR_X          0x37  /**< 角速度 X 轴 (int16, 大端) */
#define HI14_REG_GYR_Y          0x38  /**< 角速度 Y 轴 (int16, 大端) */
#define HI14_REG_GYR_Z          0x39  /**< 角速度 Z 轴 (int16, 大端) */
#define HI14_REG_MAG_X          0x3A  /**< 磁强度 X (int16, 大端) */
#define HI14_REG_MAG_Y          0x3B  /**< 磁强度 Y (int16, 大端) */
#define HI14_REG_MAG_Z          0x3C  /**< 磁强度 Z (int16, 大端) */
#define HI14_REG_ROLL_HI        0x3D  /**< Roll 角 高 16 位 (低地址存高字) */
#define HI14_REG_ROLL_LO        0x3E  /**< Roll 角 低 16 位 */
#define HI14_REG_PITCH_HI       0x3F  /**< Pitch 角 高 16 位 (低地址存高字) */
#define HI14_REG_PITCH_LO       0x40  /**< Pitch 角 低 16 位 */
#define HI14_REG_YAW_HI         0x41  /**< Yaw 角 高 16 位 (低地址存高字) */
#define HI14_REG_YAW_LO         0x42  /**< Yaw 角 低 16 位 */
#define HI14_REG_TEMP           0x43  /**< 温度 (int16, 大端) */

/** 传感器数据寄存器块 */
#define HI14_REG_SENSOR_START   0x34  /**< 传感器数据起始寄存器 */
#define HI14_REG_SENSOR_COUNT   16    /**< 传感器数据寄存器数量 */

/** 自动水平校准寄存器 */
#define HI14_REG_AUTO_LEVEL     0x00A5  /**< 自动水平校准命令寄存器 */
#define HI14_AUTO_LEVEL_VALUE   0x0003  /**< 自动水平校准触发值 */

/* ==========================================================================
 * 传感器数据比例因子
 * ========================================================================== */

#define HI14_SCALE_ACC          0.00048828f  /**< 加速度比例因子 (原始值 → G) */
#define HI14_SCALE_GYR          0.061035f    /**< 角速度比例因子 (原始值 → deg/s) */
#define HI14_SCALE_EULER        0.001f       /**< 欧拉角比例因子 (原始值 → deg) */
#define HI14_SCALE_TEMP         0.01f        /**< 温度比例因子 (原始值 → °C) */

/* ==========================================================================
 * 航向锁定参数
 * ========================================================================== */

#define HEADING_CORRECTION_MAX  30.0f   /**< 航向修正角速度最大值 (deg/s) */
#define HEADING_CORRECTION_MIN  -30.0f  /**< 航向修正角速度最小值 (deg/s) */
#define HEADING_INTEGRAL_MAX    10.0f  /**< 积分项上限 (防饱和) */
#define HEADING_INTEGRAL_MIN    -10.0f /**< 积分项下限 (防饱和) */
#define LOWPASS_DEFAULT_ALPHA   0.7f    /**< 一阶低通滤波默认系数 (0~1) */

/* ==========================================================================
 * 旋转控制参数
 * ========================================================================== */

#define ROTATE_DEADZONE          0.1f  /**< 旋转指令死区 (rad/s), 低于此值视为停止旋转 */
#define HEADING_LOCK_DEADZONE    0.05f  /**< 航向锁定触发死区 (有旋转指令时不锁定) */

/* ==========================================================================
 * 急停保护参数
 * ========================================================================== */

#define EMERGENCY_ANGLE_MAX     30.0f   /**< 急停触发角度阈值 (deg) */

/* ==========================================================================
 * 数据读取周期 (用于任务调度)
 * ========================================================================== */

#define HI14_READ_PERIOD_MS     10      /**< 传感器数据读取周期 (ms) */

/* ==========================================================================
 * 数据枚举
 * ========================================================================== */

/**
  * @brief  HI14 驱动错误码
  */
typedef enum {
    HI14_OK               = 0,  /**< 操作成功 */
    HI14_ERR_INIT         = 1,  /**< 初始化失败 */
    HI14_ERR_READ         = 2,  /**< 传感器读取失败 */
    HI14_ERR_WRITE        = 3,  /**< 寄存器写入失败 */
    HI14_ERR_CRC          = 4,  /**< CRC 校验错误 */
    HI14_ERR_TIMEOUT      = 5,  /**< 通信超时 */
    HI14_ERR_NOT_INIT     = 6,  /**< 驱动未初始化 */
} HI14_Error_t;

/* ==========================================================================
 * 数据结构
 * ========================================================================== */

/**
  * @brief  HI14 传感器数据结构体
  * @note   所有角度单位为度 (deg), 角速度单位为 deg/s,
  *         加速度单位为 G, 温度单位为 °C
  */
#pragma pack(push, 1)
typedef struct {
    /* 加速度 (G) */
    float acc_x;
    float acc_y;
    float acc_z;

    /* 角速度 (deg/s) */
    float gyr_x;
    float gyr_y;
    float gyr_z;

    /* 欧拉角 (deg) */
    float roll;
    float pitch;
    float yaw;

    /* 温度 (°C) */
    float temperature;

} HI14_SensorData_t;
#pragma pack(pop)

/* ==========================================================================
 * 全局变量声明
 * ========================================================================== */

extern HI14_SensorData_t g_imu_data;          /**< 最新 IMU 传感器数据 */
extern volatile uint8_t  g_emergency_stop;    /**< 急停标志 (1=急停激活) */

/* ==========================================================================
 * API 函数声明
 * ========================================================================== */

/**
  * @brief  初始化 HI14 驱动 (Modbus 通信层 + DMA 接收)
  * @retval HI14_Error_t 错误码
  *
  * @note   内部调用 Modbus_Init(&huart3, &g_modbus_inst)
  *         USART3 必须已由 CubeMX 初始化 (MX_USART3_UART_Init)
  */
HI14_Error_t HI14_Init(void);

/**
  * @brief  自动水平校准: 将当前 Roll/Pitch 归零
  * @retval HI14_Error_t 错误码
  *
  * @note   发送指令: 50 06 00 A5 00 03 54 69
  *         写入寄存器 0x00A5 的值为 0x0003 触发校准
  *         校准后 HI14 将当前姿态设为新的零参考面
  */
HI14_Error_t HI14_AutoLevel(void);

/**
  * @brief  批量读取所有传感器数据 (16 个寄存器, 0x34 ~ 0x43)
  * @param  data  输出: 传感器数据结构体指针 (调用者分配)
  * @retval HI14_Error_t 错误码
  *
  * @note   内部流程:
  *          1. 发送 FC 0x03 读取 16 个寄存器
  *          2. 解析 32 字节大端数据
  *          3. 应用比例因子转换为物理量
  *          4. 对欧拉角应用一阶低通滤波 (alpha = 0.3)
  *          5. 检查急停条件 (|roll| > 25° 或 |pitch| > 25°)
  */
HI14_Error_t HI14_ReadAllSensors(HI14_SensorData_t *data);

/**
  * @brief  一阶低通滤波器
  * @param  new_val  新采样值
  * @param  old_val  上一次滤波输出值
  * @param  alpha    滤波系数 (0~1, 越小滤波越强, 典型值 0.3)
  * @retval 滤波后的值
  *
  * @note   公式: output = alpha * new_val + (1 - alpha) * old_val
  *         用于减少麦轮底盘振动引入的高频噪声
  */
float LowPass_Filter(float new_val, float old_val, float alpha);

/**
  * @brief  麦轮底盘航向锁定修正计算 (PID控制)
  * @param  target_yaw  目标航向角 (deg, 范围 -180° ~ 180°)
  * @param  Kp          比例系数 (建议值 0.5 ~ 2.0)
  * @param  Ki          积分系数 (建议值 0.001 ~ 0.05, 消除稳态误差)
  * @param  Kd          微分系数 (建议值 0.01 ~ 0.2, 抑制超调/震荡)
  * @retval 角速度修正量 (deg/s), 限幅 ±30°/s
  *
  * @note   计算逻辑:
  *          1. 计算当前 Yaw 与目标 Yaw 的误差
  *          2. 将误差归一化到 [-180°, 180°]
  *          3. P项 = Kp × error, I项 = Ki × integral, D项 = Kd × (error - last_error)
  *          4. 积分限幅防饱和 (HEADING_INTEGRAL_MIN ~ HEADING_INTEGRAL_MAX)
  *          5. 限幅到 [HEADING_CORRECTION_MIN, HEADING_CORRECTION_MAX]
  */
float Mecanum_HeadingCorrection(float target_yaw, float Kp, float Ki, float Kd);

/**
  * @brief  航向角归一化到 [-180°, 180°]
  * @param  angle  输入角度 (任意范围)
  * @retval 归一化后的角度 [-180°, 180°]
  */
float Normalize_Angle(float angle);

/**
  * @brief  获取当前 Yaw 角 (滤波后)
  * @retval 当前航向角 (deg)
  */
float HI14_GetYaw(void);

/**
  * @brief  获取急停状态
  * @retval 0 = 正常, 非 0 = 急停激活
  *
  * @note   急停条件: |Roll| > 30° 或 |Pitch| > 30°
  *         由 HI14_ReadAllSensors() 内部更新
  */
uint8_t HI14_GetEmergencyStop(void);

/**
  * @brief  获取 HI14 错误码的可读描述
  * @param  err  错误码
  * @retval 错误描述字符串
  */
const char *HI14_ErrorString(HI14_Error_t err);

#ifdef __cplusplus
}
#endif

#endif /* HI14_DRIVER_H */