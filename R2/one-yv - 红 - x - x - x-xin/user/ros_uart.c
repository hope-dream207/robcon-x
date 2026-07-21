/**
  * @file    ros_uart.c
  * @brief   ROS串口通信 —— DMA Circular + IDLE中断 + FIFO + 状态机解包
  *
  *  架构:
  *   1. DMA Circular 单缓冲: 连续接收, 不丢数据
  *   2. USART IDLE 中断: 检测帧间隔, 实时将收到数据压入 FIFO
  *   3. FIFO 解耦: ISR 写 FIFO, 任务读 FIFO, 互不阻塞
  *   4. 状态机解包: 逐字节解析
  *   5. 离线检测: 200ms 无数据即判定通信丢失
  *   6. 协议保持: 0xAA + 39B 数据 + 偶校验 + 0xEE, 与上位机兼容
  */

#include "ros_uart.h"
#include "fifo.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ====== DMA 单缓冲 (Circular) ====== */
static uint8_t ros_dma_buf[ROS_DMA_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint16_t ros_dma_last_pos = 0;

/* ====== FIFO (ISR → Task) ====== */
static FIFO_TypeDef  ros_fifo;

/* ====== 解包状态机 ====== */
static ROS_UnpackData ros_unpack;

/* ====== 解析结果 ====== */
ROS_UartFrame     ROS_latest_frame;
volatile uint8_t ros_frame_ready = 0;

/* ====== 离线检测 ====== */
static volatile uint32_t ros_last_rx_tick = 0;

/* ====== 偶校验 (与上位机协议兼容) ====== */
static uint8_t calc_even_parity(const uint8_t *data, uint16_t len) {
    uint8_t p = 0;
    for (uint16_t i = 0; i < len; i++) {
        for (int j = 0; j < 8; j++) {
            p ^= (data[i] >> j) & 0x01;
        }
    }
    return p;
}

/* ====== 帧解析 ====== */
static void ROS_ParseFrame(const uint8_t *buf) {
    uint8_t recv_parity = buf[ROS_PKT_DATA_SIZE + 1];
    if (calc_even_parity(&buf[1], ROS_PKT_DATA_SIZE) != recv_parity) {
        return;
    }

    memcpy(&ROS_latest_frame.linear_vel, &buf[1], 4);
    memcpy(&ROS_latest_frame.lateral_vel, &buf[5], 4);
    memcpy(&ROS_latest_frame.angular_vel, &buf[9], 4);
    ROS_latest_frame.nav_state = buf[13];
    ROS_latest_frame.goal_reached = buf[14];
    ROS_latest_frame.grip_run = buf[15];
    ROS_latest_frame.lift_run = buf[16];
    ROS_latest_frame.rotate_run = buf[17];
    ROS_latest_frame.arm_how = buf[18];
    memcpy(&ROS_latest_frame.camera_y, &buf[19], 4);
    memcpy(&ROS_latest_frame.odom_yaw, &buf[23], 4);
    memcpy(&ROS_latest_frame.reserved_float1, &buf[27], 4);
    memcpy(&ROS_latest_frame.reserved_float2, &buf[31], 4);
    memcpy(&ROS_latest_frame.reserved_float3, &buf[35], 4);
    ROS_latest_frame.reserved_uint8 = buf[39];
    ROS_latest_frame.valid = 1;
    ros_frame_ready = 1;
}

/* ====== 状态机解包 (逐字节, FIFO_Pop 加临界区保护) ====== */
void ROS_DataUnpack(void) {
    uint8_t byte;

    while (1) {
        taskENTER_CRITICAL();
        uint8_t got = FIFO_Pop(&ros_fifo, &byte, 1);
        taskEXIT_CRITICAL();

        if (!got) break;

        switch (ros_unpack.step) {

        case ROS_STEP_HEADER:
            if (byte == ROS_PKT_HEADER) {
                ros_unpack.packet_buf[0] = byte;
                ros_unpack.index = 1;
                ros_unpack.step = ROS_STEP_DATA;
            }
            break;

        case ROS_STEP_DATA:
            ros_unpack.packet_buf[ros_unpack.index++] = byte;
            if (ros_unpack.index >= ROS_PKT_DATA_SIZE + 1) {
                ros_unpack.step = ROS_STEP_PARITY;
            }
            break;

        case ROS_STEP_PARITY:
            ros_unpack.packet_buf[ros_unpack.index++] = byte;
            ros_unpack.step = ROS_STEP_FOOTER;
            break;

        case ROS_STEP_FOOTER:
            ros_unpack.packet_buf[ros_unpack.index] = byte;
            if (byte == ROS_PKT_FOOTER) {
                ROS_ParseFrame(ros_unpack.packet_buf);
            }
            memset(&ros_unpack, 0, sizeof(ros_unpack));
            break;

        default:
            memset(&ros_unpack, 0, sizeof(ros_unpack));
            break;
        }
    }
}

/* ====== 初始化 ====== */
void ROS_UartInit(void) {
    memset(&ROS_latest_frame, 0, sizeof(ROS_UartFrame));
    ros_frame_ready = 0;

    memset(&ros_unpack, 0, sizeof(ros_unpack));

    FIFO_Init(&ros_fifo);

    ros_dma_last_pos = 0;

    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_DMA(&huart1, ros_dma_buf, ROS_DMA_BUF_SIZE);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    ros_last_rx_tick = HAL_GetTick();
}

/* ====== 串口错误后重启DMA(不清空FIFO和状态机) ====== */
void ROS_UartRestart(void) {
    ros_dma_last_pos = 0;

    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_DMA(&huart1, ros_dma_buf, ROS_DMA_BUF_SIZE);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
}

/* ====== USART1 IDLE 中断处理 (DMA Circular 单缓冲) ====== */
void ROS_UART_IRQHandler(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1) return;
    if (!(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))) return;

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    DMA_HandleTypeDef *hdma = huart->hdmarx;
    uint16_t curr_pos = ROS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(hdma);

    if (curr_pos != ros_dma_last_pos) {
        if (curr_pos > ros_dma_last_pos) {
            FIFO_Push(&ros_fifo, &ros_dma_buf[ros_dma_last_pos],
                      curr_pos - ros_dma_last_pos);
        } else {
            FIFO_Push(&ros_fifo, &ros_dma_buf[ros_dma_last_pos],
                      ROS_DMA_BUF_SIZE - ros_dma_last_pos);
            if (curr_pos > 0) {
                FIFO_Push(&ros_fifo, &ros_dma_buf[0], curr_pos);
            }
        }
        ros_dma_last_pos = curr_pos;
    }

    ros_last_rx_tick = HAL_GetTick();
}

/* ====== 获取最新帧 ====== */
uint8_t ROS_GetFrame(ROS_UartFrame *frame) {
    if (ros_frame_ready) {
        *frame = ROS_latest_frame;
        ros_frame_ready = 0;
        return 1;
    }
    return 0;
}

/* ====== 离线检测 ====== */
bool ROS_IsOffline(void) {
    return (HAL_GetTick() - ros_last_rx_tick) > ROS_OFFLINE_TIME_MS;
}