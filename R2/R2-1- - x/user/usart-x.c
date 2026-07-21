/* ==========================================================================
 * HAL UART 回调
 * ========================================================================== */

 
#include "l1_laser.h"
#include "usart.h"
#include "unitree.h"
#include "ros_uart.h"
#include "modbus.h"
#include <string.h>
extern uint8_t Temp_buffer[64];
extern UART_HandleTypeDef huart6;

/* ======== YS-IRTM 红外模块 (USART4, 仅中断接收, 3字节帧) ======== */
#define IR_ADDR1_MATCH    0x01
#define IR_ADDR2_MATCH    0x02
#define IR_FRAME_LEN      3

static uint8_t  ir_rx_byte       = 0;
static uint8_t  ir_buf[3]        = {0};
static uint8_t  ir_idx           = 0;
static volatile uint8_t  ir_frame_ready = 0;
static volatile uint8_t  ir_last_key    = 0;
static volatile uint32_t ir_last_recv_time = 0;

void YS_IRTM_Init(void)
{
    ir_idx = 0;
    ir_frame_ready = 0;
    ir_last_key = 0;
    ir_last_recv_time = HAL_GetTick();
    HAL_UART_AbortReceive(&huart4);
    (void)HAL_UART_Receive_IT(&huart4, &ir_rx_byte, 1);
}

uint8_t YS_IRTM_GetKey(void)
{
    if (ir_frame_ready)
    {
        ir_frame_ready = 0;
        /* 地址匹配才返回键值，不匹配返回 0xFF */
        if (ir_buf[0] == IR_ADDR1_MATCH && ir_buf[1] == IR_ADDR2_MATCH)
        {
            return ir_last_key;
        }
        return 0xFF;
    }
    /* 无新帧 → 返回 0xFF */
    return 0xFF;
}

uint8_t YS_IRTM_HasFrame(void)
{
    return ir_frame_ready;
}

uint32_t YS_IRTM_GetLastRecvTime(void)
{
    return ir_last_recv_time;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        l1_parse_byte(s_rx_byte);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&s_rx_byte, 1);
    }
    else if (huart->Instance == UART4) {
        ir_buf[ir_idx] = ir_rx_byte;
        ir_idx++;
        if (ir_idx >= IR_FRAME_LEN) {
            ir_idx = 0;
            ir_last_key = ir_buf[2];
            ir_frame_ready = 1;
            ir_last_recv_time = HAL_GetTick();
        }
        (void)HAL_UART_Receive_IT(&huart4, &ir_rx_byte, 1);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)Size;
    // USART6 由 M8010_CommTask 手动收发，不经过 DMA 回调
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        ROS_UartRestart();
    }
    else if (huart->Instance == USART2) {
        L1_Restart();
    }
    else if (huart->Instance == USART3) {
        extern Modbus_Instance_t g_modbus_inst;
        Modbus_Restart(&g_modbus_inst);
    }
    else if (huart->Instance == USART6) {
        // M8010_CommTask 手动收发，不重启 DMA（避免抢 RX 数据）
        (void)HAL_UART_AbortReceive(&huart6);
    }
    else if (huart->Instance == UART4) {
        (void)HAL_UART_AbortReceive(&huart4);
        ir_idx = 0;
        (void)HAL_UART_Receive_IT(&huart4, &ir_rx_byte, 1);
    }
}