/**
  * @file    hi14_driver.c
  * @brief   HiPNUC HI14 系列 IMU 高层驱动实现
  *
  * @details
  *
  *   通信参数: USART3, 115200, 8N1, 从机地址 0x50
  */

#include "hi14_driver.h"
#include "modbus.h"
#include "usart.h"
#include <math.h>    /* fabsf */
#include <string.h>  /* memset */
#include <stdbool.h>

/* ==========================================================================
 * 全局变量
 * ========================================================================== */

HI14_SensorData_t  g_imu_data;           /**< 最新 IMU 传感器数据 (全局) */
volatile uint8_t   g_emergency_stop = 0; /**< 急停标志: 0=正常, 1=急停 */

/* ==========================================================================
 * 模块内部静态变量
 * ========================================================================== */

static uint8_t  s_initialized = 0;       /**< 驱动初始化标志 */

/* 一阶低通滤波: 上一次输出值 (用于欧拉角平滑) */
static float    s_roll_filtered  = 0.0f;
static float    s_pitch_filtered = 0.0f;
static float    s_yaw_filtered   = 0.0f;
static uint8_t  s_filter_first   = 1;    /**< 首次采样标志 (跳过滤波) */

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
  * @brief  从大端字节序缓冲区读取 int16 值
  * @param  buf  字节缓冲区 (至少 offset+1 有效)
  * @param  offset  起始偏移
  * @retval 有符号 16 位整数值
  */
static inline int16_t read_int16_be(const uint8_t *buf, uint16_t offset)
{
    return (int16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

/**
  * @brief  从大端字节序缓冲区读取 int32 值 (由连续 2 个 16 位寄存器组合)
  * @param  buf     字节缓冲区
  * @param  offset  第一个寄存器(低地址)的字节偏移
  * @retval 有符号 32 位整数值
  *
  * @note   HI14 官方手册寄存器命名 R_H/L、P_H/L、Y_H/L:
  *         低地址寄存器存高 16 位, 高地址寄存器存低 16 位。
  *         组合方式: (reg[offset]   << 16) | reg[offset+2]
  *         其中 reg[offset..offset+1] 为高字, reg[offset+2..offset+3] 为低字。
  */
static inline int32_t read_int32_be_2reg(const uint8_t *buf, uint16_t offset)
{
    uint16_t high = ((uint16_t)buf[offset]     << 8) | (uint16_t)buf[offset + 1];
    uint16_t low  = ((uint16_t)buf[offset + 2] << 8) | (uint16_t)buf[offset + 3];
    return (int32_t)(((uint32_t)high << 16) | (uint32_t)low);
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
  * @brief  初始化 HI14 驱动
  *
  * @note   调用 Modbus_Init() 配置 USART3 DMA + IDLE 接收。
  *         必须在 FreeRTOS 调度器启动后调用 (使用了 HAL_Delay)。
  */
HI14_Error_t HI14_Init(void)
{
    HAL_StatusTypeDef hal_status;

    /* 绑定 USART3 到 Modbus 全局实例 (PB10-TX, PB11-RX, RS-485) */
    hal_status = Modbus_Init(&huart3, &g_modbus_inst);
    if (hal_status != HAL_OK) {
        return HI14_ERR_INIT;
    }

    /* 清零传感器数据 */
    memset(&g_imu_data, 0, sizeof(HI14_SensorData_t));

    /* 复位滤波器状态 */
    s_roll_filtered  = 0.0f;
    s_pitch_filtered = 0.0f;
    s_yaw_filtered   = 0.0f;
    s_filter_first   = 1;

    /* 清除急停标志 */
    g_emergency_stop = 0;

    s_initialized = 1;

    return HI14_OK;
}

/* ==========================================================================
 * 一阶低通滤波器
 * ========================================================================== */

/**
  * @brief  一阶低通滤波器
  * @param  new_val  新采样值
  * @param  old_val  上一次滤波输出值
  * @param  alpha    滤波系数 (0~1)
  *                   - alpha 越小, 滤波越强, 响应越慢
  *                   - alpha = 1.0 时等同于不过滤 (output = new_val)
  *                   - 典型值: 0.3 (用于麦轮底盘振动环境)
  * @retval 滤波后的值
  *
  * @note   差分方程: y[n] = α·x[n] + (1-α)·y[n-1]
  *         截止频率: fc = α / (2π·Ts·(1-α))
  *         当 Ts = 10ms, α = 0.7 时, fc ≈ 37 Hz
  */
float LowPass_Filter(float new_val, float old_val, float alpha)
{
    /* 参数限幅 */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    return alpha * new_val + (1.0f - alpha) * old_val;
}

/* ==========================================================================
 * 航向锁定修正
 * ========================================================================== */

/**
  * @brief  麦轮底盘航向锁定角速度修正量计算
  *
  * @note   航向锁定原理:
  *         麦轮底盘在平移运动时, 由于轮子打滑、地面不平等因素,
  *         实际航向会逐渐漂移。此函数根据 IMU 实测航向与目标航向的
  *         偏差, 输出一个叠加到底盘角速度指令上的修正量,
  *         使底盘自动纠正航向漂移。
  *
  *         误差归一化:
  *         例如: 当前 Yaw = 170°, 目标 Yaw = -170°
  *         直接减法 = 170 - (-170) = 340°
  *         归一化后 = 340 - 360 = -20° (最短路径)
  */
float Mecanum_HeadingCorrection(float target_yaw, float Kp, float Ki, float Kd)
{
    float error;
    float correction;
    static float last_error = 0.0f;
    static float integral = 0.0f;
    float d_error;

    error = target_yaw - g_imu_data.yaw;

    while (error > 180.0f) {
        error -= 360.0f;
    }
    while (error < -180.0f) {
        error += 360.0f;
    }

    d_error = error - last_error;
    while (d_error > 180.0f) {
        d_error -= 360.0f;
    }
    while (d_error < -180.0f) {
        d_error += 360.0f;
    }

    integral += error;
    if (integral > HEADING_INTEGRAL_MAX) {
        integral = HEADING_INTEGRAL_MAX;
    } else if (integral < HEADING_INTEGRAL_MIN) {
        integral = HEADING_INTEGRAL_MIN;
    }

    last_error = error;

    correction = Kp * error + Ki * integral + Kd * d_error;

    if (correction > HEADING_CORRECTION_MAX) {
        correction = HEADING_CORRECTION_MAX;
    } else if (correction < HEADING_CORRECTION_MIN) {
        correction = HEADING_CORRECTION_MIN;
    }

    return correction;
}

/* ==========================================================================
 * 航向角辅助函数
 * ========================================================================== */

/**
  * @brief  航向角归一化到 [-180°, 180°]
  * @param  angle  输入角度 (任意范围)
  * @retval 归一化后的角度 [-180°, 180°]
  *
  * @note   处理 ±180° 跨越问题, 保证取最短旋转路径
  */
float Normalize_Angle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
  * @brief  获取当前 Yaw 角 (滤波后)
  * @retval 当前航向角 (deg)
  */
float HI14_GetYaw(void)
{
    return g_imu_data.yaw;
}

/* ==========================================================================
 * 急停状态查询
 * ========================================================================== */

/**
  * @brief  获取急停状态
  * @retval 0 = 正常, 1 = 急停激活
  */
uint8_t HI14_GetEmergencyStop(void)
{
    return g_emergency_stop;
}

/* ==========================================================================
 * 自动水平校准
 * ========================================================================== */

/**
  * @brief  自动水平校准: 将当前 Roll/Pitch 归零
  *
  * @note   发送指令: 50 06 00 A5 00 03 54 69
  *         其中 CRC16(0x50, 0x06, 0x00, 0xA5, 0x00, 0x03) = 0x5469
  *
  *         使用场景:
  *         - 系统上电后, IMU 安装面不一定完全水平
  *         - 调用此函数将当前姿态设为"零位"
  *         - 后续读取的 Roll/Pitch 即为相对此零位的倾角
  *
  *         注意: 校准期间传感器应保持静止!
  */
HI14_Error_t HI14_AutoLevel(void)
{
    Modbus_Error_t modbus_err;

    if (!s_initialized) {
        return HI14_ERR_NOT_INIT;
    }

    /*
     * 发送自动水平校准指令:
     *   从机地址: 0x50
     *   功能码:   0x06 (写单个寄存器)
     *   寄存器:   0x00A5
     *   写入值:   0x0003 (触发校准)
     */
    modbus_err = Modbus_WriteSingleReg(&g_modbus_inst,
                                        MODBUS_DEFAULT_SLAVE_ADDR,
                                        HI14_REG_AUTO_LEVEL,
                                        HI14_AUTO_LEVEL_VALUE);

    if (modbus_err != MODBUS_OK) {
        /* 将 Modbus 错误码映射为 HI14 错误码 */
        switch (modbus_err) {
        case MODBUS_ERR_TIMEOUT:   return HI14_ERR_TIMEOUT;
        case MODBUS_ERR_CRC:       return HI14_ERR_CRC;
        default:                   return HI14_ERR_WRITE;
        }
    }

    /*
     * 校准完成后, 复位滤波器历史值,
     * 避免旧值污染后续滤波输出。
     */
    s_roll_filtered  = 0.0f;
    s_pitch_filtered = 0.0f;
    s_yaw_filtered   = 0.0f;
    s_filter_first   = 1;

    return HI14_OK;
}

/* ==========================================================================
 * 批量传感器读取
 * ========================================================================== */

/**
  * @brief  批量读取 HI14 所有传感器数据 (16 个寄存器, 0x34 ~ 0x43)
  *
  * @note   数据布局 (32 字节, 大端序, 依据 HI14 官方寄存器表):
 *
 *         偏移    寄存器    内容          类型    比例因子      单位
 *         -----------------------------------------------------------
 *         [0-1]   0x34     Acc X         int16   0.00048828    G
 *         [2-3]   0x35     Acc Y         int16   0.00048828    G
 *         [4-5]   0x36     Acc Z         int16   0.00048828    G
 *         [6-7]   0x37     Gyr X         int16   0.061035      deg/s
 *         [8-9]   0x38     Gyr Y         int16   0.061035      deg/s
 *        [10-11]  0x39     Gyr Z         int16   0.061035      deg/s
 *        [12-13]  0x3A     Mag X         int16   0.030517      uT
 *        [14-15]  0x3B     Mag Y         int16   0.030517      uT
 *        [16-17]  0x3C     Mag Z         int16   0.030517      uT
 *        [18-21]  0x3D-3E  Roll  (H/L)   int32   0.001         deg
 *        [22-25]  0x3F-40  Pitch (H/L)   int32   0.001         deg
 *        [26-29]  0x41-42  Yaw   (H/L)   int32   0.001         deg
 *        [30-31]  0x43     Temperature   int16   0.01          °C
 *
 *         注: 32 位欧拉角由 2 个连续寄存器组成, 低地址存高 16 位。
 */
HI14_Error_t HI14_ReadAllSensors(HI14_SensorData_t *data)
{
    uint8_t        raw[32];       /* 16 个寄存器 × 2 字节 = 32 字节 */
    uint16_t       rx_len = 0;
    Modbus_Error_t modbus_err;
    float          raw_roll, raw_pitch, raw_yaw;

    if (!s_initialized) {
        return HI14_ERR_NOT_INIT;
    }

    if (data == NULL) {
        return HI14_ERR_READ;
    }

    /* ---- 1. 发送 FC 0x03 读取 16 个寄存器 ---- */
    modbus_err = Modbus_ReadHoldingRegs(&g_modbus_inst,
                                         MODBUS_DEFAULT_SLAVE_ADDR,
                                         HI14_REG_SENSOR_START,
                                         HI14_REG_SENSOR_COUNT,
                                         raw,
                                         &rx_len);

    if (modbus_err != MODBUS_OK) {
        switch (modbus_err) {
        case MODBUS_ERR_TIMEOUT: return HI14_ERR_TIMEOUT;
        case MODBUS_ERR_CRC:     return HI14_ERR_CRC;
        default:                 return HI14_ERR_READ;
        }
    }

    /* 检查数据长度: 16 个寄存器 × 2 字节 = 32 字节 */
    if (rx_len < 32) {
        return HI14_ERR_READ;
    }

    /* ---- 2. 解析大端数据 ---- */

    /* 加速度 (int16 → float, 单位 G) */
    data->acc_x = (float)read_int16_be(raw, 0)  * HI14_SCALE_ACC;
    data->acc_y = (float)read_int16_be(raw, 2)  * HI14_SCALE_ACC;
    data->acc_z = (float)read_int16_be(raw, 4)  * HI14_SCALE_ACC;

    /* 角速度 (int16 → float, 单位 deg/s) */
    data->gyr_x = (float)read_int16_be(raw, 6)  * HI14_SCALE_GYR;
    data->gyr_y = (float)read_int16_be(raw, 8)  * HI14_SCALE_GYR;
    data->gyr_z = (float)read_int16_be(raw, 10) * HI14_SCALE_GYR;

    /* 欧拉角 (2 寄存器组合为 int32 → float, 单位 deg)
     * 官方布局: Roll=0x3D-0x3E, Pitch=0x3F-0x40, Yaw=0x41-0x42
     * 低地址存高 16 位, 高地址存低 16 位 */
    raw_roll  = (float)read_int32_be_2reg(raw, 18) * HI14_SCALE_EULER;
    raw_pitch = (float)read_int32_be_2reg(raw, 22) * HI14_SCALE_EULER;
    raw_yaw   = (float)read_int32_be_2reg(raw, 26) * HI14_SCALE_EULER;

    /* 温度 (int16 → float, 单位 °C) */
    data->temperature = (float)read_int16_be(raw, 30) * HI14_SCALE_TEMP;

    /* ---- 3. 欧拉角一阶低通滤波 ---- */
    if (s_filter_first) {
        /* 首次采样: 直接用原始值初始化滤波器 */
        s_roll_filtered  = raw_roll;
        s_pitch_filtered = raw_pitch;
        s_yaw_filtered   = raw_yaw;
        s_filter_first   = 0;
    } else {
        s_roll_filtered  = LowPass_Filter(raw_roll,  s_roll_filtered,  LOWPASS_DEFAULT_ALPHA);
        s_pitch_filtered = LowPass_Filter(raw_pitch, s_pitch_filtered, LOWPASS_DEFAULT_ALPHA);
        // s_yaw_filtered   = LowPass_Filter(raw_yaw,   s_yaw_filtered,   LOWPASS_DEFAULT_ALPHA);
        /* yaw 滤波: 先归一化角度差到 [-180°,180°], 取最短旋转路径,
         * 避免跨越 ±180° 时 (如 +179° → -179°) 线性插值跳变到 0° 附近 */
        float yaw_diff = raw_yaw - s_yaw_filtered;
        while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
        while (yaw_diff < -180.0f) yaw_diff += 360.0f;
        s_yaw_filtered = s_yaw_filtered + LOWPASS_DEFAULT_ALPHA * yaw_diff;
    }

    data->roll  = s_roll_filtered;
    data->pitch = s_pitch_filtered;
    data->yaw   = s_yaw_filtered;

    /* ---- 4. 急停检测 ---- */
    if (fabsf(data->roll) > EMERGENCY_ANGLE_MAX ||
        fabsf(data->pitch) > EMERGENCY_ANGLE_MAX) {
        g_emergency_stop = 1;
    } else {
        g_emergency_stop = 0;
    }

    return HI14_OK;
}

/* ==========================================================================
 * 错误码转字符串
 * ========================================================================== */

/**
  * @brief  获取 HI14 错误码的可读描述
  */
const char *HI14_ErrorString(HI14_Error_t err)
{
    switch (err) {
    case HI14_OK:           return "HI14_OK";
    case HI14_ERR_INIT:     return "HI14_ERR_INIT";
    case HI14_ERR_READ:     return "HI14_ERR_READ";
    case HI14_ERR_WRITE:    return "HI14_ERR_WRITE";
    case HI14_ERR_CRC:      return "HI14_ERR_CRC";
    case HI14_ERR_TIMEOUT:  return "HI14_ERR_TIMEOUT";
    case HI14_ERR_NOT_INIT: return "HI14_ERR_NOT_INIT";
    default:                return "HI14_UNKNOWN";
    }
}