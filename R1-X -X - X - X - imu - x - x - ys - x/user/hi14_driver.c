/**
  * @file    hi14_driver.c
  * @brief   HiPNUC HI14 系列 IMU 高层驱动实现
  *
  * @details
  *
  *   通信参数: USART1, 115200, 8N1, 从机地址 0x50
  */

#include "hi14_driver.h"
#include "modbus.h"
#include "usart.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

HI14_SensorData_t  g_imu_data;
volatile uint8_t   g_emergency_stop = 0;

static uint8_t  s_initialized = 0;

static float    s_roll_filtered  = 0.0f;
static float    s_pitch_filtered = 0.0f;
static float    s_yaw_filtered   = 0.0f;
static uint8_t  s_filter_first   = 1;

static inline int16_t read_int16_be(const uint8_t *buf, uint16_t offset)
{
    return (int16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

static inline int32_t read_int32_be_2reg(const uint8_t *buf, uint16_t offset)
{
    uint16_t high = ((uint16_t)buf[offset]     << 8) | (uint16_t)buf[offset + 1];
    uint16_t low  = ((uint16_t)buf[offset + 2] << 8) | (uint16_t)buf[offset + 3];
    return (int32_t)(((uint32_t)high << 16) | (uint32_t)low);
}

HI14_Error_t HI14_Init(void)
{
    HAL_StatusTypeDef hal_status;

    hal_status = Modbus_Init(&huart1, &g_modbus_inst);
    if (hal_status != HAL_OK) {
        return HI14_ERR_INIT;
    }

    memset(&g_imu_data, 0, sizeof(HI14_SensorData_t));

    s_roll_filtered  = 0.0f;
    s_pitch_filtered = 0.0f;
    s_yaw_filtered   = 0.0f;
    s_filter_first   = 1;

    g_emergency_stop = 0;

    s_initialized = 1;

    return HI14_OK;
}

float LowPass_Filter(float new_val, float old_val, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    return alpha * new_val + (1.0f - alpha) * old_val;
}

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

float HI14_GetYaw(void)
{
    return g_imu_data.yaw;
}

uint8_t HI14_GetEmergencyStop(void)
{
    return g_emergency_stop;
}

HI14_Error_t HI14_AutoLevel(void)
{
    Modbus_Error_t modbus_err;

    if (!s_initialized) {
        return HI14_ERR_NOT_INIT;
    }

    modbus_err = Modbus_WriteSingleReg(&g_modbus_inst,
                                        MODBUS_DEFAULT_SLAVE_ADDR,
                                        HI14_REG_AUTO_LEVEL,
                                        HI14_AUTO_LEVEL_VALUE);

    if (modbus_err != MODBUS_OK) {
        switch (modbus_err) {
        case MODBUS_ERR_TIMEOUT:   return HI14_ERR_TIMEOUT;
        case MODBUS_ERR_CRC:       return HI14_ERR_CRC;
        default:                   return HI14_ERR_WRITE;
        }
    }

    s_roll_filtered  = 0.0f;
    s_pitch_filtered = 0.0f;
    s_yaw_filtered   = 0.0f;
    s_filter_first   = 1;

    return HI14_OK;
}

HI14_Error_t HI14_ReadAllSensors(HI14_SensorData_t *data)
{
    uint8_t        raw[32];
    uint16_t       rx_len = 0;
    Modbus_Error_t modbus_err;
    float          raw_roll, raw_pitch, raw_yaw;

    if (!s_initialized) {
        return HI14_ERR_NOT_INIT;
    }

    if (data == NULL) {
        return HI14_ERR_READ;
    }

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

    if (rx_len < 32) {
        return HI14_ERR_READ;
    }

    data->acc_x = (float)read_int16_be(raw, 0)  * HI14_SCALE_ACC;
    data->acc_y = (float)read_int16_be(raw, 2)  * HI14_SCALE_ACC;
    data->acc_z = (float)read_int16_be(raw, 4)  * HI14_SCALE_ACC;

    data->gyr_x = (float)read_int16_be(raw, 6)  * HI14_SCALE_GYR;
    data->gyr_y = (float)read_int16_be(raw, 8)  * HI14_SCALE_GYR;
    data->gyr_z = (float)read_int16_be(raw, 10) * HI14_SCALE_GYR;

    raw_roll  = (float)read_int32_be_2reg(raw, 18) * HI14_SCALE_EULER;
    raw_pitch = (float)read_int32_be_2reg(raw, 22) * HI14_SCALE_EULER;
    raw_yaw   = (float)read_int32_be_2reg(raw, 26) * HI14_SCALE_EULER;

    data->temperature = (float)read_int16_be(raw, 30) * HI14_SCALE_TEMP;

    if (s_filter_first) {
        s_roll_filtered  = raw_roll;
        s_pitch_filtered = raw_pitch;
        s_yaw_filtered   = raw_yaw;
        s_filter_first   = 0;
    } else {
        s_roll_filtered  = LowPass_Filter(raw_roll,  s_roll_filtered,  LOWPASS_DEFAULT_ALPHA);
        s_pitch_filtered = LowPass_Filter(raw_pitch, s_pitch_filtered, LOWPASS_DEFAULT_ALPHA);
        float yaw_diff = raw_yaw - s_yaw_filtered;
        while (yaw_diff > 180.0f)  yaw_diff -= 360.0f;
        while (yaw_diff < -180.0f) yaw_diff += 360.0f;
        s_yaw_filtered = s_yaw_filtered + LOWPASS_DEFAULT_ALPHA * yaw_diff;
    }

    data->roll  = s_roll_filtered;
    data->pitch = s_pitch_filtered;
    data->yaw   = s_yaw_filtered;

    if (fabsf(data->roll) > EMERGENCY_ANGLE_MAX ||
        fabsf(data->pitch) > EMERGENCY_ANGLE_MAX) {
        g_emergency_stop = 1;
    } else {
        g_emergency_stop = 0;
    }

    return HI14_OK;
}

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