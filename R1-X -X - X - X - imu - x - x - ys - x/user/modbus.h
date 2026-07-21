/**
  * @file    modbus.h
  * @brief   Modbus RTU 主机协议栈 (RS-485 自动流向控制)
  *
  * @details
  *
  *   硬件约束:
  *     - RS-485 模块为硬件自动流向控制, 严禁操作 RE/DE GPIO
  *     - 发送完成后等待硬件切换为接收状态
  *
  *   依赖:
  *     - STM32F4xx HAL 库
  *     - FreeRTOS (用于超时和临界区保护)
  */

#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_FC_READ_HOLDING_REGS   0x03
#define MODBUS_FC_WRITE_SINGLE_REG    0x06

#define MODBUS_DEFAULT_SLAVE_ADDR     0x50

#define MODBUS_TIMEOUT_MS             20

#define MODBUS_RX_BUF_MAX             260

#define MODBUS_DMA_BUF_SIZE           512

typedef enum {
    MODBUS_OK             = 0,
    MODBUS_ERR_TIMEOUT    = 1,
    MODBUS_ERR_CRC        = 2,
    MODBUS_ERR_ADDR       = 3,
    MODBUS_ERR_FUNC       = 4,
    MODBUS_ERR_EXCEPTION  = 5,
    MODBUS_ERR_LENGTH     = 6,
    MODBUS_ERR_BUSY       = 7,
    MODBUS_ERR_HAL        = 8,
} Modbus_Error_t;

typedef struct {
    UART_HandleTypeDef   *huart;
    DMA_HandleTypeDef     hdma_rx;
    uint8_t               dma_buf[MODBUS_DMA_BUF_SIZE] __attribute__((aligned(4)));
    volatile uint16_t     dma_last_pos;
    volatile uint8_t      response_ready;
    volatile uint8_t      rx_timeout_flag;
    uint8_t               rx_buf[MODBUS_RX_BUF_MAX];
    volatile uint16_t     rx_len;
    uint8_t               slave_addr;
    volatile uint32_t     last_rx_tick;
} Modbus_Instance_t;

extern Modbus_Instance_t g_modbus_inst;

uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len);

HAL_StatusTypeDef Modbus_Init(UART_HandleTypeDef *huart, Modbus_Instance_t *inst);

Modbus_Error_t Modbus_ReadHoldingRegs(Modbus_Instance_t *inst,
                                       uint8_t slave_addr,
                                       uint16_t start_reg,
                                       uint16_t count,
                                       uint8_t *rx_data,
                                       uint16_t *rx_data_len);

Modbus_Error_t Modbus_WriteSingleReg(Modbus_Instance_t *inst,
                                      uint8_t slave_addr,
                                      uint16_t reg_addr,
                                      uint16_t reg_value);

void Modbus_UART_IDLE_Callback(Modbus_Instance_t *inst);

void Modbus_Restart(Modbus_Instance_t *inst);

const char *Modbus_ErrorString(Modbus_Error_t err);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */