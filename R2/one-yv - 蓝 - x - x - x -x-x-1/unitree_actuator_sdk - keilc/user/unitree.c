#include "unitree.h"
#include "usart.h"
#include "gom_protocol.h"
#include <string.h>
#include <math.h>

extern UART_HandleTypeDef huart6;

MotorCmd_t cmd[MOTOR_NUM];
MotorData_t data[MOTOR_NUM];

static float motor_pos_zero_offsets[MOTOR_NUM] = {0.0f, 0.0f, 0.0f};
static uint8_t motor_zero_calibrated[MOTOR_NUM] = {0, 0, 0};

#define CALIB_START_COUNT  400U
#define CALIB_FORCE_COUNT  900U

static uint32_t motor_tick_count = 0;

uint8_t Unitree_IsCalibrated(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (!motor_zero_calibrated[i]) return 0;
    }
    return 1;
}

void Unitree_Init(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        MotorData_init(&data[i]);

        cmd[i].id = i;
        cmd[i].mode = 1;
        cmd[i].K_P = 0.0f;
        cmd[i].K_W = 0.0f;
        cmd[i].Pos = 0.0f;
        cmd[i].W = 0.0f;
        cmd[i].T = 0.0f;

        modify_data(&cmd[i]);
    }
}

void Unitree_SetMotor(uint8_t motor_id, float q, float dq, float tau, float kp, float kd)
{
    if (motor_id >= MOTOR_NUM) return;

    cmd[motor_id].id = motor_id;
    cmd[motor_id].mode = 1;

    if (motor_tick_count < CALIB_FORCE_COUNT) {
        cmd[motor_id].K_P = 0.0f;
        cmd[motor_id].K_W = 0.0f;
        cmd[motor_id].T   = 0.0f;
        cmd[motor_id].Pos = motor_pos_zero_offsets[motor_id];
        cmd[motor_id].W   = 0.0f;
    } else {
        cmd[motor_id].K_P = kp;
        cmd[motor_id].K_W = kd;
        cmd[motor_id].Pos = q + motor_pos_zero_offsets[motor_id];
        cmd[motor_id].W   = dq;
        cmd[motor_id].T   = tau;
    }

    modify_data(&cmd[motor_id]);
    MotorData_calculate_error(&data[motor_id], cmd[motor_id].tar_pos, cmd[motor_id].tar_w, cmd[motor_id].tar_t);
}

void Unitree_Control(void)
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {
        modify_data(&cmd[i]);
    }
}

void Unitree_OnUart6Rx(const uint8_t *buf, size_t len)
{
    if (!buf || len < sizeof(RIS_MotorData_t)) return;

    const uint8_t *frame = NULL;
    for (size_t i = 0; i + 1U < len; i++)
    {
        if (buf[i] == 0xFDU && buf[i + 1U] == 0xEEU)
        {
            if (i + sizeof(RIS_MotorData_t) <= len)
            {
                frame = &buf[i];
            }
            break;
        }
    }

    if (!frame) return;

    MotorData_t temp;
    MotorData_init(&temp);
    memcpy(&temp.motor_recv_data, frame, sizeof(RIS_MotorData_t));
    extract_data(&temp);

    if (!temp.correct) return;

    uint8_t id = temp.motor_id;
    if (id >= MOTOR_NUM) return;

    data[id] = temp;

    motor_tick_count++;

    if (motor_tick_count >= CALIB_START_COUNT && !motor_zero_calibrated[id]) {
        if (id == 1) {
            motor_pos_zero_offsets[id] = data[id].Pos - 1.30f;
        } else {
            motor_pos_zero_offsets[id] = data[id].Pos;
        }
        motor_zero_calibrated[id] = 1;
    }
}

float Unitree_GetJointAngle(uint8_t motor_id)
{
    if (motor_id >= MOTOR_NUM) return 0.0f;
    return data[motor_id].Pos - motor_pos_zero_offsets[motor_id];
}