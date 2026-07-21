/**
  * @file    modbus.h
  * @brief   Modbus RTU 主机协议栈 (RS-485 自动流向控制)
  *
  * @details
  *   功能:
  *     - Modbus CRC16 计算 (多项式 0x8005, 初始值 0xFFFF)
  *     - 功能码 0x03 (读保持寄存器) 请求/响应
  *     - 功能码 0x06 (写单个寄存器) 请求/响应
  *     - DMA 环形缓冲 + UART 空闲中断 (IDLE) 不定长接收
  *     - 超时检测 (基于 FreeRTOS 系统节拍)
  *
  *   硬件约束:
  *     - RS-485 模块为硬件自动流向控制, 严禁操作 RE/DE GPIO
  *     - 发送完成后 HAL_Delay(1) 等待硬件切换为接收状态
  *
  *   依赖:
  *     - STM32F4xx HAL 库
  *     - FreeRTOS (用于超时和临界区保护)
  *     - user/fifo.h (环形缓冲区)
  */

#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 宏定义
 * ========================================================================== */

/** Modbus 功能码 */
#define MODBUS_FC_READ_HOLDING_REGS   0x03  /**< 读保持寄存器 */
#define MODBUS_FC_WRITE_SINGLE_REG    0x06  /**< 写单个寄存器 */

/** Modbus 异常码 (从机响应功能码最高位置位时, 第3字节为异常码) */
#define MODBUS_EXC_ILLEGAL_FUNCTION    0x01  /**< 功能码错误 */
#define MODBUS_EXC_ILLEGAL_ADDRESS     0x02  /**< 起始地址错误 */
#define MODBUS_EXC_ILLEGAL_VALUE       0x03  /**< 寄存器数量错误 */
#define MODBUS_EXC_SLAVE_FAILURE       0x04  /**< 寄存器值错误 */
#define MODBUS_EXC_CRC_ERROR          0x05  /**< CRC错误 */
#define MODBUS_EXC_DEVICE_BUSY        0x06  /**< 设备繁忙 */

/** 默认从机地址 (HI14 出厂默认) */
#define MODBUS_DEFAULT_SLAVE_ADDR     0x50

/** 超时时间 (ms), 基于 FreeRTOS 系统节拍 */
#define MODBUS_TIMEOUT_MS             20

/** 最大响应帧长度 (地址1 + 功能码1 + 字节计数1 + 数据252 + CRC2 = 257, 取整) */
#define MODBUS_RX_BUF_MAX             260

/** DMA 环形缓冲区大小 (需大于最大响应帧) */
#define MODBUS_DMA_BUF_SIZE           512

/** FIFO 缓冲区大小 */
#define MODBUS_FIFO_SIZE              1024

/* ==========================================================================
 * 枚举定义
 * ========================================================================== */

/**
  * @brief  Modbus 操作错误码
  */
typedef enum {
    MODBUS_OK             = 0,  /**< 操作成功 */
    MODBUS_ERR_TIMEOUT    = 1,  /**< 响应超时 (从机无应答) */
    MODBUS_ERR_CRC        = 2,  /**< CRC16 校验错误 */
    MODBUS_ERR_ADDR       = 3,  /**< 从机地址不匹配 */
    MODBUS_ERR_FUNC       = 4,  /**< 功能码不匹配 */
    MODBUS_ERR_EXCEPTION  = 5,  /**< 从机返回异常码 */
    MODBUS_ERR_LENGTH     = 6,  /**< 响应长度异常 */
    MODBUS_ERR_BUSY       = 7,  /**< 总线忙 (上一次传输未完成) */
    MODBUS_ERR_HAL        = 8,  /**< HAL 底层发送/接收错误 */
} Modbus_Error_t;

/* ==========================================================================
 * 结构体定义
 * ========================================================================== */

/**
  * @brief  Modbus RTU 主机实例结构体
  * @note   每个 UART 外设对应一个实例, 支持多从机总线
  */
typedef struct {
    UART_HandleTypeDef   *huart;               /**< 绑定的 UART 句柄 */
    DMA_HandleTypeDef     hdma_rx;             /**< DMA 接收句柄 (内部管理) */
    uint8_t               dma_buf[MODBUS_DMA_BUF_SIZE] __attribute__((aligned(4))); /**< DMA 环形缓冲区 */
    volatile uint16_t     dma_last_pos;        /**< 上次 DMA 读取位置 */
    volatile uint8_t      response_ready;      /**< 响应就绪标志 (ISR 置位, 任务清除) */
    volatile uint8_t      rx_timeout_flag;     /**< 接收超时标志 */
    uint8_t               rx_buf[MODBUS_RX_BUF_MAX]; /**< 解析后的响应帧缓冲区 */
    volatile uint16_t     rx_len;              /**< 响应帧实际长度 */
    uint8_t               slave_addr;          /**< 当前通信的从机地址 */
    volatile uint8_t      last_exception_code; /**< 最近一次异常响应的异常码 (0x01~0x06) */
    volatile uint32_t     last_rx_tick;        /**< 最后一次收到数据的系统节拍 */
} Modbus_Instance_t;

/* ==========================================================================
 * 全局实例声明 (供中断服务函数访问)
 * ========================================================================== */

extern Modbus_Instance_t g_modbus_inst;        /**< 全局 Modbus 实例 (绑定 USART3, HI14 IMU) */

/* ==========================================================================
 * API 函数声明
 * ========================================================================== */

/**
  * @brief  计算 Modbus RTU CRC16
  * @param  buf    数据缓冲区指针
  * @param  len    数据长度 (字节数)
  * @retval 16 位 CRC 值 (低字节在前, 符合 Modbus 帧序)
  *
  * @note   多项式: 0x8005, 初始值: 0xFFFF, LSB-first 反射
  *         例如: 数据 [0x01, 0x03, 0x00, 0x00, 0x00, 0x01] 的 CRC 为 0x840A
  */
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len);

/**
  * @brief  初始化 Modbus 主机实例并启动 DMA 接收
  * @param  huart  绑定的 UART 句柄指针 (如 &huart2)
  * @param  inst   要初始化的 Modbus 实例指针
  * @retval HAL_OK = 成功, 其他 = HAL 错误码
  *
  * @note   内部操作:
  *          1. 配置 DMA 流为对应 UART_RX 环形接收
  *          2. 启用对应 UART 全局中断 + IDLE 中断
  *          3. 初始化 FIFO 和状态变量
  *          4. 启动 DMA 接收
  *         UART 的 GPIO 和基本参数由 CubeMX 生成的 MX_USARTx_UART_Init() 完成
  */
HAL_StatusTypeDef Modbus_Init(UART_HandleTypeDef *huart, Modbus_Instance_t *inst);

/**
  * @brief  读保持寄存器 (功能码 0x03)
  * @param  inst        Modbus 实例指针
  * @param  slave_addr  从机地址 (HI14 默认为 0x50)
  * @param  start_reg   起始寄存器地址 (16 位)
  * @param  count       读取寄存器数量 (1~125)
  * @param  rx_data     输出: 接收到的寄存器数据缓冲区 (调用者分配, 至少 count*2 字节)
  * @param  rx_data_len 输出: 实际接收到的数据字节数
  * @retval Modbus_Error_t 错误码 (MODBUS_OK = 成功)
  *
  * @note   响应格式: [addr][0x03][byte_cnt][data...][CRC_LO][CRC_HI]
  *         阻塞等待直到收到响应或超时
  */
Modbus_Error_t Modbus_ReadHoldingRegs(Modbus_Instance_t *inst,
                                       uint8_t slave_addr,
                                       uint16_t start_reg,
                                       uint16_t count,
                                       uint8_t *rx_data,
                                       uint16_t *rx_data_len);

/**
  * @brief  写单个寄存器 (功能码 0x06)
  * @param  inst        Modbus 实例指针
  * @param  slave_addr  从机地址
  * @param  reg_addr    寄存器地址 (16 位)
  * @param  reg_value   寄存器值 (16 位)
  * @retval Modbus_Error_t 错误码 (MODBUS_OK = 成功)
  *
  * @note   响应格式: 从机回显请求帧 [addr][0x06][reg_HI][reg_LO][val_HI][val_LO][CRC]
  */
Modbus_Error_t Modbus_WriteSingleReg(Modbus_Instance_t *inst,
                                      uint8_t slave_addr,
                                      uint16_t reg_addr,
                                      uint16_t reg_value);

/**
  * @brief  UART IDLE 中断回调处理函数
  * @param  inst  Modbus 实例指针
  * @note   在 USART2_IRQHandler 中调用此函数
  *         检测 IDLE 标志 → 计算 DMA 接收字节数 → 将数据从 DMA 缓冲区复制到 rx_buf
  */
void Modbus_UART_IDLE_Callback(Modbus_Instance_t *inst);

/**
  * @brief  重启 Modbus DMA 接收 (UART 错误后恢复)
  * @param  inst  Modbus 实例指针
  * @note   在 HAL_UART_ErrorCallback 中 USART2 错误时调用
  *         仅重启 DMA + IDLE 中断, 不清零实例配置
  */
void Modbus_Restart(Modbus_Instance_t *inst);

/**
  * @brief  获取最后一次错误的可读字符串描述
  * @param  err  错误码
  * @retval 错误描述字符串
  */
const char *Modbus_ErrorString(Modbus_Error_t err);

const char *Modbus_ExceptionCodeString(uint8_t exc_code);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_H */