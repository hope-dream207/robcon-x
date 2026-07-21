/**
  * @file    modbus.c
  * @brief   Modbus RTU 主机协议栈实现 (RS-485 自动流向控制)
  *
  * @details
  *   架构:
  *   硬件约束:
  *     - 严禁操作 RE/DE GPIO (RS-485 模块为硬件自动流向控制)
  *     - USART1_RX 绑定到 DMA2_Stream2_Channel4 (环形模式, CubeMX 已配置)
  */

#include "modbus.h"
#include "usart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

static const uint16_t modbus_crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

Modbus_Instance_t g_modbus_inst;

uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc = (crc >> 8) ^ modbus_crc16_table[(crc ^ *buf++) & 0xFF];
    }

    return crc;
}

HAL_StatusTypeDef Modbus_Init(UART_HandleTypeDef *huart, Modbus_Instance_t *inst)
{
    if (huart == NULL || inst == NULL) {
        return HAL_ERROR;
    }

    memset(inst, 0, sizeof(Modbus_Instance_t));
    inst->huart = huart;
    inst->slave_addr = MODBUS_DEFAULT_SLAVE_ADDR;

    if (huart->hdmarx == NULL) {
        return HAL_ERROR;
    }

    {
        IRQn_Type irqn;
        if (huart->Instance == USART1)      irqn = USART1_IRQn;
        else if (huart->Instance == USART2) irqn = USART2_IRQn;
        else if (huart->Instance == USART3) irqn = USART3_IRQn;
        else if (huart->Instance == USART6) irqn = USART6_IRQn;
        else return HAL_ERROR;

        if (NVIC_GetEnableIRQ(irqn) == 0) {
            HAL_NVIC_SetPriority(irqn, 6, 0);
            HAL_NVIC_EnableIRQ(irqn);
        }
    }

    HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_DMA(huart, inst->dma_buf, MODBUS_DMA_BUF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    inst->dma_last_pos    = 0;
    inst->response_ready  = 0;
    inst->rx_len          = 0;
    inst->last_rx_tick    = HAL_GetTick();

    return HAL_OK;
}

void Modbus_Restart(Modbus_Instance_t *inst)
{
    if (inst == NULL || inst->huart == NULL) {
        return;
    }

    inst->dma_last_pos   = 0;
    inst->response_ready = 0;
    inst->rx_len         = 0;

    HAL_UART_AbortReceive(inst->huart);

    if (HAL_UART_Receive_DMA(inst->huart, inst->dma_buf,
                             MODBUS_DMA_BUF_SIZE) != HAL_OK) {
        return;
    }

    __HAL_DMA_DISABLE_IT(inst->huart->hdmarx, DMA_IT_HT);

    __HAL_UART_ENABLE_IT(inst->huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(inst->huart);

    inst->last_rx_tick = HAL_GetTick();
}

static HAL_StatusTypeDef modbus_send_frame(Modbus_Instance_t *inst,
                                            uint8_t addr,
                                            uint8_t func,
                                            const uint8_t *data,
                                            uint16_t len)
{
    uint8_t tx_buf[264];
    uint16_t tx_len;
    uint16_t crc;
    HAL_StatusTypeDef status;

    if (inst == NULL || inst->huart == NULL) {
        return HAL_ERROR;
    }

    tx_buf[0] = addr;
    tx_buf[1] = func;

    if (data != NULL && len > 0) {
        memcpy(&tx_buf[2], data, len);
    }

    tx_len = 2 + len;

    crc = Modbus_CRC16(tx_buf, tx_len);

    tx_buf[tx_len]     = (uint8_t)(crc & 0xFF);
    tx_buf[tx_len + 1] = (uint8_t)((crc >> 8) & 0xFF);
    tx_len += 2;

    if (inst->huart->RxState != HAL_UART_STATE_BUSY_RX) {
        Modbus_Restart(inst);
    }

    inst->rx_len         = 0;
    inst->response_ready = 0;

    status = HAL_UART_Transmit(inst->huart, tx_buf, tx_len, MODBUS_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    {
        uint32_t tc_start = HAL_GetTick();
        while (__HAL_UART_GET_FLAG(inst->huart, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - tc_start) > 2) {
                break;
            }
        }
    }

    return HAL_OK;
}

static Modbus_Error_t modbus_wait_response(Modbus_Instance_t *inst,
                                            uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (1) {
        if (inst->response_ready) {
            return MODBUS_OK;
        }

        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return MODBUS_ERR_TIMEOUT;
        }

        vTaskDelay(1);
    }
}

static Modbus_Error_t modbus_validate_response(Modbus_Instance_t *inst,
                                                uint8_t slave_addr,
                                                uint8_t func)
{
    uint16_t rx_len = inst->rx_len;
    uint8_t  *rx_buf = inst->rx_buf;

    if (rx_len < 4) {
        return MODBUS_ERR_LENGTH;
    }

    if (rx_buf[0] != slave_addr) {
        return MODBUS_ERR_ADDR;
    }

    if (rx_buf[1] & 0x80) {
        return MODBUS_ERR_EXCEPTION;
    }

    if (rx_buf[1] != func) {
        return MODBUS_ERR_FUNC;
    }

    {
        uint16_t crc_calc = Modbus_CRC16(rx_buf, rx_len);
        if (crc_calc != 0x0000) {
            return MODBUS_ERR_CRC;
        }
    }

    return MODBUS_OK;
}

void Modbus_UART_IDLE_Callback(Modbus_Instance_t *inst)
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma;
    uint16_t            curr_pos;
    uint16_t            new_bytes;
    uint16_t            copy_len;

    if (inst == NULL || inst->huart == NULL) {
        return;
    }

    huart = inst->huart;

    if (!(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))) {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    hdma = huart->hdmarx;
    if (hdma == NULL) {
        return;
    }

    curr_pos = MODBUS_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(hdma);

    if (curr_pos >= inst->dma_last_pos) {
        new_bytes = curr_pos - inst->dma_last_pos;
    } else {
        new_bytes = MODBUS_DMA_BUF_SIZE - inst->dma_last_pos + curr_pos;
    }

    if (new_bytes == 0) {
        return;
    }

    copy_len   = 0;

    if (curr_pos > inst->dma_last_pos) {
        uint16_t len = curr_pos - inst->dma_last_pos;
        if (len > MODBUS_RX_BUF_MAX) len = MODBUS_RX_BUF_MAX;
        memcpy(&inst->rx_buf[0], &inst->dma_buf[inst->dma_last_pos], len);
        copy_len = len;
    } else {
        uint16_t len1 = MODBUS_DMA_BUF_SIZE - inst->dma_last_pos;
        uint16_t len2 = curr_pos;

        if (len1 > MODBUS_RX_BUF_MAX) len1 = MODBUS_RX_BUF_MAX;
        memcpy(&inst->rx_buf[0], &inst->dma_buf[inst->dma_last_pos], len1);
        copy_len = len1;

        if (len2 > 0 && copy_len < MODBUS_RX_BUF_MAX) {
            uint16_t remaining = MODBUS_RX_BUF_MAX - copy_len;
            if (len2 > remaining) len2 = remaining;
            memcpy(&inst->rx_buf[copy_len], &inst->dma_buf[0], len2);
            copy_len += len2;
        }
    }

    inst->dma_last_pos = curr_pos;

    inst->rx_len = copy_len;
    inst->response_ready = 1;
    inst->last_rx_tick = HAL_GetTick();
}

Modbus_Error_t Modbus_ReadHoldingRegs(Modbus_Instance_t *inst,
                                       uint8_t slave_addr,
                                       uint16_t start_reg,
                                       uint16_t count,
                                       uint8_t *rx_data,
                                       uint16_t *rx_data_len)
{
    uint8_t       tx_data[4];
    Modbus_Error_t err;
    uint8_t        expected_byte_cnt;

    if (inst == NULL || rx_data == NULL || rx_data_len == NULL) {
        return MODBUS_ERR_LENGTH;
    }

    if (count == 0 || count > 125) {
        return MODBUS_ERR_LENGTH;
    }

    tx_data[0] = (uint8_t)((start_reg >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(start_reg & 0xFF);
    tx_data[2] = (uint8_t)((count >> 8) & 0xFF);
    tx_data[3] = (uint8_t)(count & 0xFF);

    if (modbus_send_frame(inst, slave_addr, MODBUS_FC_READ_HOLDING_REGS,
                          tx_data, 4) != HAL_OK) {
        return MODBUS_ERR_HAL;
    }

    err = modbus_wait_response(inst, MODBUS_TIMEOUT_MS);
    if (err != MODBUS_OK) {
        return err;
    }

    err = modbus_validate_response(inst, slave_addr, MODBUS_FC_READ_HOLDING_REGS);
    if (err != MODBUS_OK) {
        return err;
    }

    expected_byte_cnt = (uint8_t)(count * 2);

    if (inst->rx_len < (uint16_t)(3 + expected_byte_cnt + 2)) {
        return MODBUS_ERR_LENGTH;
    }

    if (inst->rx_buf[2] != expected_byte_cnt) {
        return MODBUS_ERR_LENGTH;
    }

    *rx_data_len = expected_byte_cnt;
    memcpy(rx_data, &inst->rx_buf[3], expected_byte_cnt);

    return MODBUS_OK;
}

Modbus_Error_t Modbus_WriteSingleReg(Modbus_Instance_t *inst,
                                      uint8_t slave_addr,
                                      uint16_t reg_addr,
                                      uint16_t reg_value)
{
    uint8_t       tx_data[4];
    Modbus_Error_t err;

    if (inst == NULL) {
        return MODBUS_ERR_LENGTH;
    }

    tx_data[0] = (uint8_t)((reg_addr >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(reg_addr & 0xFF);
    tx_data[2] = (uint8_t)((reg_value >> 8) & 0xFF);
    tx_data[3] = (uint8_t)(reg_value & 0xFF);

    if (modbus_send_frame(inst, slave_addr, MODBUS_FC_WRITE_SINGLE_REG,
                          tx_data, 4) != HAL_OK) {
        return MODBUS_ERR_HAL;
    }

    err = modbus_wait_response(inst, MODBUS_TIMEOUT_MS);
    if (err != MODBUS_OK) {
        return err;
    }

    err = modbus_validate_response(inst, slave_addr, MODBUS_FC_WRITE_SINGLE_REG);
    if (err != MODBUS_OK) {
        return err;
    }

    return MODBUS_OK;
}

const char *Modbus_ErrorString(Modbus_Error_t err)
{
    switch (err) {
    case MODBUS_OK:            return "OK";
    case MODBUS_ERR_TIMEOUT:   return "TIMEOUT";
    case MODBUS_ERR_CRC:       return "CRC_ERROR";
    case MODBUS_ERR_ADDR:      return "ADDR_MISMATCH";
    case MODBUS_ERR_FUNC:      return "FUNC_MISMATCH";
    case MODBUS_ERR_EXCEPTION: return "SLAVE_EXCEPTION";
    case MODBUS_ERR_LENGTH:    return "LENGTH_ERROR";
    case MODBUS_ERR_BUSY:      return "BUSY";
    case MODBUS_ERR_HAL:       return "HAL_ERROR";
    default:                   return "UNKNOWN";
    }
}