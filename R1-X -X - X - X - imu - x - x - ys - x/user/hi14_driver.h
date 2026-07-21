/**
  * @file    hi14_driver.h
  * @brief   HiPNUC HI14 系列 IMU 高层驱动 (Modbus RTU)
  *
  * @details
  *
  *   硬件:
  *     - 通信接口: USART1 (PA9-TX, PA10-RX) @ 115200, 8N1
  *     - RS-485 模块: 硬件自动流向控制 (无 RE/DE GPIO)
  *     - HI14 从机地址: 0x50 (出厂默认)
  */

#ifndef HI14_DRIVER_H
#define HI14_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HI14_REG_ACC_X          0x34
#define HI14_REG_ACC_Y          0x35
#define HI14_REG_ACC_Z          0x36
#define HI14_REG_GYR_X          0x37
#define HI14_REG_GYR_Y          0x38
#define HI14_REG_GYR_Z          0x39
#define HI14_REG_MAG_X          0x3A
#define HI14_REG_MAG_Y          0x3B
#define HI14_REG_MAG_Z          0x3C
#define HI14_REG_ROLL_HI        0x3D
#define HI14_REG_ROLL_LO        0x3E
#define HI14_REG_PITCH_HI       0x3F
#define HI14_REG_PITCH_LO       0x40
#define HI14_REG_YAW_HI         0x41
#define HI14_REG_YAW_LO         0x42
#define HI14_REG_TEMP           0x43

#define HI14_REG_SENSOR_START   0x34
#define HI14_REG_SENSOR_COUNT   16

#define HI14_REG_AUTO_LEVEL     0x00A5
#define HI14_AUTO_LEVEL_VALUE   0x0003

#define HI14_SCALE_ACC          0.00048828f
#define HI14_SCALE_GYR          0.061035f
#define HI14_SCALE_EULER        0.001f
#define HI14_SCALE_TEMP         0.01f

#define HEADING_CORRECTION_MAX  30.0f
#define HEADING_CORRECTION_MIN  -30.0f
#define HEADING_INTEGRAL_MAX    10.0f
#define HEADING_INTEGRAL_MIN    -10.0f
#define LOWPASS_DEFAULT_ALPHA   0.7f

#define ROTATE_DEADZONE          0.1f
#define HEADING_LOCK_DEADZONE    0.05f

#define EMERGENCY_ANGLE_MAX     30.0f

#define HI14_READ_PERIOD_MS     10

typedef enum {
    HI14_OK               = 0,
    HI14_ERR_INIT         = 1,
    HI14_ERR_READ         = 2,
    HI14_ERR_WRITE        = 3,
    HI14_ERR_CRC          = 4,
    HI14_ERR_TIMEOUT      = 5,
    HI14_ERR_NOT_INIT     = 6,
} HI14_Error_t;

#pragma pack(push, 1)
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;

    float gyr_x;
    float gyr_y;
    float gyr_z;

    float roll;
    float pitch;
    float yaw;

    float temperature;

} HI14_SensorData_t;
#pragma pack(pop)

extern HI14_SensorData_t g_imu_data;
extern volatile uint8_t  g_emergency_stop;

HI14_Error_t HI14_Init(void);

HI14_Error_t HI14_AutoLevel(void);

HI14_Error_t HI14_ReadAllSensors(HI14_SensorData_t *data);

float LowPass_Filter(float new_val, float old_val, float alpha);

float Mecanum_HeadingCorrection(float target_yaw, float Kp, float Ki, float Kd);

float Normalize_Angle(float angle);

float HI14_GetYaw(void);

uint8_t HI14_GetEmergencyStop(void);

const char *HI14_ErrorString(HI14_Error_t err);

#ifdef __cplusplus
}
#endif

#endif /* HI14_DRIVER_H */