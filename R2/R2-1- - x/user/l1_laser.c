/**
  * @file    l1_laser.c
  * @brief   L1s-40 激光测距模组驱动 (HEX 协议, USART2)
  *
  * @details
  *   HEX 协议通信:
  *     - 连续测量命令: A5 5A 03 00 FC
  *     - 单次测量命令: A5 5A 02 00 FD
  *     - 停止测量命令: A5 5A 05 00 FA
  *     - 响应帧头: B4 69 04 + 4字节大端序距离 + 校验
  *     - 距离单位: mm, /1000 转米
  *
  *   硬件连接: USART2 PA2(TX) / PA3(RX), 115200 8N1
  *   接收方式: 中断逐字节 + 状态机解析
  */

#include "l1_laser.h"
#include "usart.h"
#include "unitree.h"
#include "ros_uart.h"
#include "modbus.h"
#include <string.h>

/* ==========================================================================
 * 内部状态
 * ========================================================================== */

static uint8_t s_initialized = 0;

Modbus_Instance_t g_l1_modbus_inst;

int value[2] = {0, 0};

/* ==========================================================================
 * HEX 协议接收状态机
 * ========================================================================== */

#define L1_HEX_HEADER     0xB46904
#define L1_HEX_DATA_LEN   4

#define L1_OUTLIER_THRESH 1000//相邻帧差超过1m视为毛刺，丢弃
#define L1_IIR_ALPHA      0.7f//α=0.4偏平滑，α=0.7偏响应快

volatile uint8_t  s_rx_byte = 0;
static volatile uint32_t s_header_shift = 0;
static volatile uint8_t  s_rx_state = 0;
static volatile uint8_t  s_rx_data_count = 0;
static volatile uint32_t s_rx_distance_raw = 0;
static volatile float    s_distance_m = 0.0f;
static volatile uint8_t  s_data_ready = 0;
static volatile int      s_last_valid = 0;
static volatile float    s_filtered = 0.0f;
static volatile uint8_t  s_first_frame = 1;

/* ==========================================================================
 * HEX 协议命令
 * ========================================================================== */

static const uint8_t cmd_continuous[5] = {0xA5, 0x5A, 0x03, 0x00, 0xFC};
static const uint8_t cmd_single[5]     = {0xA5, 0x5A, 0x02, 0x00, 0xFD};
static const uint8_t cmd_stop[5]       = {0xA5, 0x5A, 0x05, 0x00, 0xFA};

/* ==========================================================================
 * 内部函数
 * ========================================================================== */

void l1_parse_byte(uint8_t ch)
{
    if (s_rx_state == 0) {
        s_header_shift = (s_header_shift << 8) | ch;
        if ((s_header_shift & 0xFFFFFF) == L1_HEX_HEADER) {
            s_rx_state = 1;
            s_rx_data_count = 0;
            s_rx_distance_raw = 0;
        }
    } else {
        s_rx_distance_raw = (s_rx_distance_raw << 8) | ch;
        s_rx_data_count++;
        if (s_rx_data_count >= L1_HEX_DATA_LEN) {
            int raw = (int)s_rx_distance_raw;

            if (s_first_frame) {
                s_last_valid = raw;
                s_filtered = (float)raw;
                s_first_frame = 0;
            } else {
                if (raw - s_last_valid > L1_OUTLIER_THRESH ||
                    s_last_valid - raw > L1_OUTLIER_THRESH) {
                    raw = s_last_valid;
                } else {
                    s_last_valid = raw;
                }
                s_filtered = L1_IIR_ALPHA * (float)raw +
                             (1.0f - L1_IIR_ALPHA) * s_filtered;
            }

            s_distance_m = s_filtered / 1000.0f;
            value[0] = (int)s_filtered;
            s_data_ready = 1;
            s_rx_state = 0;
            s_header_shift = 0;
        }
    }
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

void L1_Init(void)
{
    HAL_UART_AbortReceive(&huart2);
    HAL_UART_AbortTransmit(&huart2);

    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        (void)huart2.Instance->DR;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart2);

    s_header_shift = 0;
    s_rx_state = 0;
    s_rx_data_count = 0;
    s_rx_distance_raw = 0;
    s_data_ready = 0;
    s_last_valid = 0;
    s_filtered = 0.0f;
    s_first_frame = 1;

    HAL_UART_Transmit(&huart2, cmd_continuous, 5, 100);

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1);

    s_initialized = 1;
}

/* ==========================================================================
 * 距离读取 (非阻塞)
 * ========================================================================== */

L1_Error_t L1_ReadDistance(float *distance_m)
{
    if (!s_initialized) {
        return L1_ERR_BUSY;
    }
    if (distance_m == NULL) {
        return L1_ERR_BUSY;
    }
    if (!s_data_ready) {
        return L1_ERR_TIMEOUT;
    }

    *distance_m = s_distance_m;
    s_data_ready = 0;
    return L1_OK;
}

/* ==========================================================================
 * 错误恢复
 * ========================================================================== */

void L1_Restart(void)
{
    HAL_UART_AbortReceive(&huart2);
    s_header_shift = 0;
    s_rx_state = 0;
    s_rx_data_count = 0;
    s_data_ready = 0;
    s_last_valid = 0;
    s_filtered = 0.0f;
    s_first_frame = 1;
    s_initialized = 0;

    L1_Init();
}

/* ==========================================================================
 * 错误码转字符串
 * ========================================================================== */

const char *L1_ErrorString(L1_Error_t err)
{
    switch (err) {
    case L1_OK:          return "OK";
    case L1_ERR_TIMEOUT: return "TIMEOUT";
    case L1_ERR_CRC:     return "CRC_ERROR";
    case L1_ERR_BUSY:    return "BUSY";
    case L1_ERR_HAL:     return "HAL_ERROR";
    default:             return "UNKNOWN";
    }
}