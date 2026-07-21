/**
  * @file    modbus.c
  * @brief   Modbus RTU 主机协议栈实现 (RS-485 自动流向控制)
  *
  * @details
  */

#include "modbus.h"
#include "usart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* ==========================================================================
 * Modbus CRC16 查找表 (多项式 0x8005, 反射算法)
 *
 * 生成公式:
 *   for i in 0..255:
 *       crc = i
 *       for j in 0..7:
 *           if crc & 1: crc = (crc >> 1) ^ 0xA001
 *           else:       crc >>= 1
 *       table[i] = crc
 *
 * 字节更新公式: crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
 * ========================================================================== */
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

/* ==========================================================================
 * 全局 Modbus 实例 (绑定 USART3, HI14 IMU)
 * ========================================================================== */
Modbus_Instance_t g_modbus_inst;

/* ==========================================================================
 * CRC16 计算
 * ========================================================================== */

/**
  * @brief  计算 Modbus RTU CRC16
  * @param  buf  数据缓冲区指针
  * @param  len  数据长度 (字节数)
  * @retval 16 位 CRC 值 (低字节在前, 可直接附加到 Modbus 帧尾)
  */
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;  /* Modbus CRC 初始值 */

    while (len--) {
        crc = (crc >> 8) ^ modbus_crc16_table[(crc ^ *buf++) & 0xFF];
    }

    return crc;  /* 低字节在前, 高字节在后 (已反射) */
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
  * @brief  初始化 Modbus 主机实例
  *
  * @note   配置流程:
  *          1. 清零实例结构体
  *          2. 若 UART 已由 CubeMX 配置 DMA (huart->hdmarx != NULL), 直接复用
  *             否则动态配置 DMA 流 (兼容未在 CubeMX 中配置 DMA 的 UART)
  *          3. 启用对应 UART 的 NVIC 中断 (若未启用)
  *          4. 启动 DMA 环形接收 + IDLE 中断
  *
   *         当前使用 USART3 (PB10-TX/PB11-RX):
 *           - 动态配置: DMA1_Stream1_Ch4 (RX Circular)
 *           - NVIC: USART3_IRQn 由 Modbus_Init 动态使能, 优先级 7
  */
HAL_StatusTypeDef Modbus_Init(UART_HandleTypeDef *huart, Modbus_Instance_t *inst)
{
    if (huart == NULL || inst == NULL) {
        return HAL_ERROR;
    }

    /* ---- 清零实例 ---- */
    memset(inst, 0, sizeof(Modbus_Instance_t));
    inst->huart = huart;
    inst->slave_addr = MODBUS_DEFAULT_SLAVE_ADDR;

    /*
     * ---- DMA 配置 ----
     * 若 CubeMX 已为 UART 配置了 DMA 接收 (huart->hdmarx != NULL),
     * 则直接复用, 无需重新初始化 DMA 流。
     * 否则动态配置 (适用于 CubeMX 中未启用 DMA 的 UART, 如 USART2)。
     */
    if (huart->hdmarx == NULL) {
        /* CubeMX 未配置 DMA — 动态配置 */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /*
         * 根据 UART 实例选择 DMA 流:
         *   USART2_RX → DMA1_Stream5_Ch4
         *   USART3_RX → DMA1_Stream1_Ch4
         */
        if (huart->Instance == USART2) {
            inst->hdma_rx.Instance                 = DMA1_Stream5;
            inst->hdma_rx.Init.Channel             = DMA_CHANNEL_4;
        } else if (huart->Instance == USART3) {
            inst->hdma_rx.Instance                 = DMA1_Stream1;
            inst->hdma_rx.Init.Channel             = DMA_CHANNEL_4;
        } else {
            return HAL_ERROR;  /* 不支持的 UART 实例 */
        }

        inst->hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        inst->hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        inst->hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
        inst->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        inst->hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        inst->hdma_rx.Init.Mode                = DMA_CIRCULAR;
        inst->hdma_rx.Init.Priority            = DMA_PRIORITY_MEDIUM;
        inst->hdma_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&inst->hdma_rx) != HAL_OK) {
            return HAL_ERROR;
        }

        __HAL_LINKDMA(huart, hdmarx, inst->hdma_rx);
    }
    /*
     * CubeMX 已配置 DMA (如 USART3) — huart->hdmarx 已指向
     * hdma_usart3_rx, 无需额外操作。直接使用即可。
     */

    /*
     * ---- NVIC 中断使能 ----
     * 若 CubeMX 已使能中断则跳过, 否则手动使能。
     * 判断方法: 检查 NVIC 是否已使能该 IRQ。
     */
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

    /* ---- 启动 DMA 环形接收 ---- */
    HAL_UART_AbortReceive(huart);
    if (HAL_UART_Receive_DMA(huart, inst->dma_buf, MODBUS_DMA_BUF_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    /* ---- 使能 UART IDLE 中断 ---- */
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* ---- 初始化状态 ---- */
    inst->dma_last_pos    = 0;
    inst->response_ready  = 0;
    inst->rx_len          = 0;
    inst->last_rx_tick    = HAL_GetTick();

    return HAL_OK;
}

/**
  * @brief  重启 Modbus DMA 接收 (UART 错误后恢复)
  * @param  inst  Modbus 实例指针
  *
  * @note   调用时机: HAL_UART_ErrorCallback 中检测到 UART 错误时
  *         仅重启 DMA 接收和 IDLE 中断, 不清零实例数据 (保留 slave_addr 等)
  *         不清空 rx_buf (避免中断上下文中操作大缓冲区)
  */
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

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
  * @brief  组装并发送 Modbus RTU 请求帧
  * @param  inst   Modbus 实例指针
  * @param  addr   从机地址
  * @param  func   功能码
  * @param  data   功能码之后的数据 (不含 CRC)
  * @param  len    数据长度
  * @retval HAL_OK / HAL_ERROR
  *
  * @note   发送帧格式: [addr][func][data...][CRC_LO][CRC_HI]
  *         发送后 HAL_Delay(1) 等待 RS-485 硬件自动切换为接收状态
  */
static HAL_StatusTypeDef modbus_send_frame(Modbus_Instance_t *inst,
                                            uint8_t addr,
                                            uint8_t func,
                                            const uint8_t *data,
                                            uint16_t len)
{
    uint8_t tx_buf[264];  /* 最大帧: 地址1 + 功能码1 + 数据252 + CRC2 = 256, 留余量 */
    uint16_t tx_len;
    uint16_t crc;
    HAL_StatusTypeDef status;

    if (inst == NULL || inst->huart == NULL) {
        return HAL_ERROR;
    }

    /* ---- 组装帧 ---- */
    tx_buf[0] = addr;    /* 从机地址 */
    tx_buf[1] = func;    /* 功能码 */

    if (data != NULL && len > 0) {
        memcpy(&tx_buf[2], data, len);
    }

    tx_len = 2 + len;    /* 不含 CRC 的长度 */

    /* 计算 CRC (对 addr + func + data 全部内容) */
    crc = Modbus_CRC16(tx_buf, tx_len);

    /* 附加 CRC (低字节在前) */
    tx_buf[tx_len]     = (uint8_t)(crc & 0xFF);       /* CRC 低字节 */
    tx_buf[tx_len + 1] = (uint8_t)((crc >> 8) & 0xFF); /* CRC 高字节 */
    tx_len += 2;

    /* ---- 发送前: 检查 DMA 接收状态, 若已停止则先恢复 ---- */
    if (inst->huart->RxState != HAL_UART_STATE_BUSY_RX) {
        Modbus_Restart(inst);
    }

    /* ---- 发送前: 清除响应标志 (先清 rx_len 再清 ready, 防 ISR 竞态) ---- */
    inst->rx_len         = 0;
    inst->response_ready = 0;

    /* ---- 阻塞发送 ---- */
    status = HAL_UART_Transmit(inst->huart, tx_buf, tx_len, MODBUS_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    /*
     * ---- RS-485 方向切换 ----
     * 硬件自动流向控制芯片在发送完成后自动切换回接收状态。
     * 显式确认 TC (发送完成) 标志, 确保最后一个字节完全移出后硬件自动切换方向。
     * DMA 接收早已启动, 不会丢失从机响应, 无需固定 1ms 延时。
     * 加 2ms 超时保护, 避免 TC 因异常未置位时死循环卡死整个读取流程。
     */
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

/**
  * @brief  等待从机响应 (轮询 response_ready 标志)
  * @param  inst     Modbus 实例指针
  * @param  timeout_ms  超时时间 (ms)
  * @retval MODBUS_OK = 收到响应, MODBUS_ERR_TIMEOUT = 超时
  *
  * @note   响应由 UART IDLE 中断触发, 在 Modbus_UART_IDLE_Callback 中
  *         将数据从 DMA 缓冲区复制到 inst->rx_buf, 并置位 response_ready。
  */
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

/**
  * @brief  验证接收到的 Modbus RTU 响应帧
  * @param  inst        Modbus 实例指针
  * @param  slave_addr  期望的从机地址
  * @param  func        期望的功能码
  * @retval Modbus_Error_t 错误码
  *
  * @note   验证项目:
  *          1. 帧长度 >= 4 字节 (addr + func + CRC2 最小)
  *          2. 从机地址匹配
  *          3. 功能码匹配 (若为异常响应, 功能码最高位置位)
  *          4. CRC16 校验
  */
static Modbus_Error_t modbus_validate_response(Modbus_Instance_t *inst,
                                                uint8_t slave_addr,
                                                uint8_t func)
{
    uint16_t rx_len = inst->rx_len;
    uint8_t  *rx_buf = inst->rx_buf;

    /* 1. 最小帧长度检查: 地址(1) + 功能码(1) + CRC(2) = 4 */
    if (rx_len < 4) {
        return MODBUS_ERR_LENGTH;
    }

    /* 2. 从机地址检查 */
    if (rx_buf[0] != slave_addr) {
        return MODBUS_ERR_ADDR;
    }

    /* 3. 异常响应检查 (功能码最高位 = 1) */
    if (rx_buf[1] & 0x80) {
        /* 异常响应格式: [addr][func|0x80][exception_code][CRC]
         * 异常码定义:
         *   0x01: 功能码错误
         *   0x02: 起始地址错误
         *   0x03: 寄存器数量错误
         *   0x04: 寄存器值错误
         *   0x05: CRC错误
         *   0x06: 设备繁忙 */
        if (rx_len >= 5) {
            inst->last_exception_code = rx_buf[2];
        } else {
            inst->last_exception_code = 0xFF;
        }
        return MODBUS_ERR_EXCEPTION;
    }

    /* 4. 功能码检查 */
    if (rx_buf[1] != func) {
        return MODBUS_ERR_FUNC;
    }

    /* 5. CRC16 校验 (对整个响应帧, 含 CRC 自身) */
    {
        uint16_t crc_calc = Modbus_CRC16(rx_buf, rx_len);
        /*
         * Modbus CRC 校验: 对包含 CRC 的完整帧重新计算, 结果应为 0x0000
         * 原理: 附加 CRC 后的完整帧, 其 CRC16 恒为 0
         */
        if (crc_calc != 0x0000) {
            return MODBUS_ERR_CRC;
        }
    }

    return MODBUS_OK;
}

/* ==========================================================================
 * UART IDLE 中断回调
 * ========================================================================== */

/**
  * @brief  USART IDLE 中断处理 (DMA 环形缓冲模式)
  * @param  inst  Modbus 实例指针
  *
  * @note   调用时机: USARTx_IRQHandler 中检测到 IDLE 标志时
  *
  *         工作原理:
  *          - DMA 在环形缓冲区中持续写入
  *          - 每次 IDLE 中断触发时, 通过 NDTR 寄存器计算本次接收的字节数
  *          - 将新数据从 DMA 缓冲区复制到 rx_buf
  *          - 置位 response_ready 通知等待的任务
  *
  *         环形缓冲处理 (参考 ros_uart.c):
  *          - curr_pos > last_pos: 未回绕, 直接复制 [last_pos, curr_pos)
  *          - curr_pos <= last_pos: 已回绕, 先复制 [last_pos, buf_end),
  *            再复制 [0, curr_pos)
  */
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

    /* 仅处理绑定 UART 的 IDLE 中断 */
    if (!(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))) {
        return;
    }

    /* 清除 IDLE 标志 (必须: 先读 SR, 再读 DR) */
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    hdma = huart->hdmarx;
    if (hdma == NULL) {
        return;
    }

    /*
     * 计算当前 DMA 写入位置:
     *   NDTR 是剩余传输计数 (递减)
     *   当前位置 = 缓冲区大小 - NDTR
     */
    curr_pos = MODBUS_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(hdma);

    /* 计算本次接收的新字节数 */
    if (curr_pos >= inst->dma_last_pos) {
        new_bytes = curr_pos - inst->dma_last_pos;
    } else {
        new_bytes = MODBUS_DMA_BUF_SIZE - inst->dma_last_pos + curr_pos;
    }

    /* 无新数据则返回 */
    if (new_bytes == 0) {
        return;
    }

    /*
     * 从 DMA 环形缓冲区复制数据到响应缓冲区。
     * 处理回绕情况: 如果 curr_pos <= last_pos,
     * 说明 DMA 写指针已经回绕到缓冲区起始位置。
     * 注意: new_bytes 已经过溢出保护, rx_buf 足够容纳。
     */
    copy_len   = 0;

    if (curr_pos > inst->dma_last_pos) {
        /* 情况 1: 未回绕 [last_pos ... curr_pos) */
        uint16_t len = curr_pos - inst->dma_last_pos;
        if (len > MODBUS_RX_BUF_MAX) len = MODBUS_RX_BUF_MAX;
        memcpy(&inst->rx_buf[0], &inst->dma_buf[inst->dma_last_pos], len);
        copy_len = len;
    } else {
        /* 情况 2: 已回绕 [last_pos ... buf_end) + [0 ... curr_pos) */
        uint16_t len1 = MODBUS_DMA_BUF_SIZE - inst->dma_last_pos;
        uint16_t len2 = curr_pos;

        /* 第一段: 缓冲区尾部 */
        if (len1 > MODBUS_RX_BUF_MAX) len1 = MODBUS_RX_BUF_MAX;
        memcpy(&inst->rx_buf[0], &inst->dma_buf[inst->dma_last_pos], len1);
        copy_len = len1;

        /* 第二段: 缓冲区头部 (如果还有空间) */
        if (len2 > 0 && copy_len < MODBUS_RX_BUF_MAX) {
            uint16_t remaining = MODBUS_RX_BUF_MAX - copy_len;
            if (len2 > remaining) len2 = remaining;
            memcpy(&inst->rx_buf[copy_len], &inst->dma_buf[0], len2);
            copy_len += len2;
        }
    }

    /* 更新 DMA 位置追踪 */
    inst->dma_last_pos = curr_pos;

    /* 存储响应长度并通知等待任务 */
    inst->rx_len = copy_len;
    inst->response_ready = 1;
    inst->last_rx_tick = HAL_GetTick();
}

/* ==========================================================================
 * 功能码 0x03: 读保持寄存器
 * ========================================================================== */

/**
  * @brief  读保持寄存器 (功能码 0x03)
  *
  * @note   请求帧: [addr][0x03][start_reg_HI][start_reg_LO][count_HI][count_LO][CRC]
  *         响应帧: [addr][0x03][byte_cnt][data...][CRC]
  *
  *         响应数据在 rx_data 中按大端序排列:
  *         寄存器 N 的高字节在前, 低字节在后。
  */
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
        return MODBUS_ERR_LENGTH;  /* Modbus 协议限制: 最多 125 个寄存器 */
    }

    /* ---- 组装请求数据 (不含地址和功能码) ---- */
    tx_data[0] = (uint8_t)((start_reg >> 8) & 0xFF);  /* 起始地址高字节 */
    tx_data[1] = (uint8_t)(start_reg & 0xFF);         /* 起始地址低字节 */
    tx_data[2] = (uint8_t)((count >> 8) & 0xFF);      /* 寄存器数量高字节 */
    tx_data[3] = (uint8_t)(count & 0xFF);             /* 寄存器数量低字节 */

    /* ---- 发送请求 ---- */
    if (modbus_send_frame(inst, slave_addr, MODBUS_FC_READ_HOLDING_REGS,
                          tx_data, 4) != HAL_OK) {
        return MODBUS_ERR_HAL;
    }

    /* ---- 等待响应 ---- */
    err = modbus_wait_response(inst, MODBUS_TIMEOUT_MS);
    if (err != MODBUS_OK) {
        return err;
    }

    /* ---- 验证响应 ---- */
    err = modbus_validate_response(inst, slave_addr, MODBUS_FC_READ_HOLDING_REGS);
    if (err != MODBUS_OK) {
        return err;
    }

    /* ---- 检查响应长度 ---- */
    expected_byte_cnt = (uint8_t)(count * 2);  /* 每个寄存器 2 字节 */

    if (inst->rx_len < (uint16_t)(3 + expected_byte_cnt + 2)) {
        /* 最小长度: addr(1) + func(1) + byte_cnt(1) + data(N) + CRC(2) */
        return MODBUS_ERR_LENGTH;
    }

    if (inst->rx_buf[2] != expected_byte_cnt) {
        return MODBUS_ERR_LENGTH;
    }

    /* ---- 提取数据 (跳过 addr, func, byte_cnt) ---- */
    *rx_data_len = expected_byte_cnt;
    memcpy(rx_data, &inst->rx_buf[3], expected_byte_cnt);

    return MODBUS_OK;
}

/* ==========================================================================
 * 功能码 0x06: 写单个寄存器
 * ========================================================================== */

/**
  * @brief  写单个寄存器 (功能码 0x06)
  *
  * @note   请求帧: [addr][0x06][reg_HI][reg_LO][val_HI][val_LO][CRC]
  *         响应帧: 从机回显请求帧 (相同内容)
  */
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

    /* ---- 组装请求数据 ---- */
    tx_data[0] = (uint8_t)((reg_addr >> 8) & 0xFF);   /* 寄存器地址高字节 */
    tx_data[1] = (uint8_t)(reg_addr & 0xFF);          /* 寄存器地址低字节 */
    tx_data[2] = (uint8_t)((reg_value >> 8) & 0xFF);  /* 寄存器值高字节 */
    tx_data[3] = (uint8_t)(reg_value & 0xFF);         /* 寄存器值低字节 */

    /* ---- 发送请求 ---- */
    if (modbus_send_frame(inst, slave_addr, MODBUS_FC_WRITE_SINGLE_REG,
                          tx_data, 4) != HAL_OK) {
        return MODBUS_ERR_HAL;
    }

    /* ---- 等待响应 ---- */
    err = modbus_wait_response(inst, MODBUS_TIMEOUT_MS);
    if (err != MODBUS_OK) {
        return err;
    }

    /* ---- 验证响应 ---- */
    err = modbus_validate_response(inst, slave_addr, MODBUS_FC_WRITE_SINGLE_REG);
    if (err != MODBUS_OK) {
        return err;
    }

    /*
     * 对于 FC 0x06, 标准从机会回显完整请求帧。
     * 可选额外检查: 验证回显的寄存器地址和值是否匹配。
     * 此处仅做 CRC 校验, 地址和值的深度校验留给调用者决定。
     */

    return MODBUS_OK;
}

/* ==========================================================================
 * 错误码转字符串
 * ========================================================================== */

/**
  * @brief  获取 Modbus 错误码的可读描述
  */
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

const char *Modbus_ExceptionCodeString(uint8_t exc_code)
{
    switch (exc_code) {
    case MODBUS_EXC_ILLEGAL_FUNCTION: return "ILLEGAL_FUNCTION(0x01)";
    case MODBUS_EXC_ILLEGAL_ADDRESS:  return "ILLEGAL_ADDRESS(0x02)";
    case MODBUS_EXC_ILLEGAL_VALUE:    return "ILLEGAL_VALUE(0x03)";
    case MODBUS_EXC_SLAVE_FAILURE:    return "SLAVE_FAILURE(0x04)";
    case MODBUS_EXC_CRC_ERROR:        return "CRC_ERROR(0x05)";
    case MODBUS_EXC_DEVICE_BUSY:      return "DEVICE_BUSY(0x06)";
    default:                           return "UNKNOWN_EXCEPTION";
    }
}