/**
  * @file    ros_uart.h
  * @brief   ROS串口通信模块 —— DMA Circular + IDLE中断 + FIFO + 状态机解包
  * @details
  *  架构:
  *    - DMA Circular 单缓冲连续接收, 不丢数据
  *    - USART IDLE 中断检测帧间隔, 实时将数据送入 FIFO
  *    - 状态机逐字节解包, 比暴力扫描帧头更健壮
  *    - 基于时间戳的离线检测, 200ms 无数据即判定离线
  *    - 协议保持与上位机的兼容: 0xAA + 39B数据 + 偶校验 + 0xEE
  */

#ifndef ROS_UART_H
#define ROS_UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 帧协议定义 (保持与上位机兼容) ---- */
#define ROS_PKT_HEADER     0xAA
#define ROS_PKT_FOOTER     0xEE
#define ROS_PKT_DATA_SIZE  39
#define ROS_PKT_TOTAL_SIZE (ROS_PKT_DATA_SIZE + 3)  /* 头 + 数据 + 校验 + 尾 = 42 */

/* ---- DMA & FIFO 配置 ---- */
#define ROS_DMA_BUF_SIZE   128   /* 单块DMA缓冲区大小 */
#define ROS_FIFO_SIZE      1024  /* FIFO环形缓冲区大小 */

/* ---- 离线判定 ---- */
#define ROS_OFFLINE_TIME_MS  200

/* ---- 解包状态机 ---- */
typedef enum {
    ROS_STEP_HEADER = 0,
    ROS_STEP_DATA,
    ROS_STEP_PARITY,
    ROS_STEP_FOOTER
} ROS_UnpackStep;

typedef struct {
    uint8_t        packet_buf[ROS_PKT_TOTAL_SIZE];
    ROS_UnpackStep step;
    uint8_t        index;
} ROS_UnpackData;

/* ---- 解析后的数据帧 (匹配上位机发送格式) ---- */
#pragma pack(push, 1)
typedef struct {
    float linear_vel;        // x线速度
    float lateral_vel;       // y线速度
    float angular_vel;       // 角速度
    uint8_t nav_state;       // 导航状态码
    uint8_t goal_reached;    // 目标是否到达
    uint8_t grip_run;        // 气抓: 0关闭/1打开
    uint8_t lift_run;         // 升降: 0下降/1上升
    uint8_t rotate_run;     // 旋转: 0停止/1旋转
    uint8_t arm_how;        // 0停止/1获取x_L/2获取x_H/3获取put/4放回
    float camera_y;          // 相机y坐标(或扩展数据)
    float odom_yaw;          // 里程计航向角
    float reserved_float1;   // 预留扩展用
    float reserved_float2;   // 预留扩展用
    float reserved_float3;   // 预留扩展用
    uint8_t reserved_uint8; // 预留扩展用
    uint8_t valid;           // 数据有效标志
} ROS_UartFrame;
#pragma pack(pop)

/* ---- 对外接口 ---- */

/**
  * @brief  初始化ROS串口通信:
  *         - 配置 DMA Circular 循环接收
  *         - 初始化 FIFO 和解包状态机
  *         - 使能 USART1 IDLE 中断
  *         - 启动 DMA 接收
  */
void ROS_UartInit(void);

/**
  * @brief  串口错误后重启DMA接收(不清空FIFO和状态机)
  */
void ROS_UartRestart(void);

/**
  * @brief  任务循环中调用: 从 FIFO 逐字节读取并执行状态机解包
  */
void ROS_DataUnpack(void);

/**
  * @brief  获取最新解析成功的帧
  * @retval 1=有新帧, 0=无新帧
  */
uint8_t ROS_GetFrame(ROS_UartFrame *frame);

/**
  * @brief  检查ROS串口是否离线
  * @retval true=离线, false=在线
  */
bool ROS_IsOffline(void);

/**
  * @brief  在 USART1_IRQHandler 中调用, 处理 IDLE 中断
  * @note   DMA Circular + IDLE检测
  */
void ROS_UART_IRQHandler(UART_HandleTypeDef *huart);

/* ---- 外部变量 ---- */
extern ROS_UartFrame       ROS_latest_frame;
extern volatile uint8_t   ros_frame_ready;

#endif