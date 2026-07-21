#include "CAN_receive.h"
#include "R2006.h"
#include "arm.h"
#include "contorl_two.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Wheel_type wheel;
extern R2006_TypeDef R2006;
motor_measure_t motor_infor;
motor_measure_t motor_infor1;

static CAN_TxHeaderTypeDef chassis_wheel_tx_message;
static uint8_t chassis_wheel_can_send_data[8];

static CAN_TxHeaderTypeDef chassis_wheelO_tx_message;
static uint8_t chassis_wheelO_can_send_data[8];

static CAN_TxHeaderTypeDef CAN2_R2006_ID_tx_message;
static uint8_t CAN2_R2006_ID_can2_send_data[8];

uint16_t bug;
// motor data read

void get_moto_measure(motor_measure_t *ptr, uint8_t data[8]) {
  ptr->last_ecd = ptr->ecd;
  ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
  ptr->speed_rpm = (int16_t)(data[2] << 8 | data[3]);
  ptr->current_raw = (int16_t)(data[4] << 8 | data[5]);
  ptr->temperate = data[6];
  if (ptr->ecd - ptr->last_ecd > 4096)
    ptr->round_cnt--;
  else if (ptr->ecd - ptr->last_ecd < -4096)
    ptr->round_cnt++;
  ptr->total_angle = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_angle;
    ptr->given_current = ptr->current_raw / 819.2f;
  ptr->real_angle1 = ptr->total_angle / 36.1935f / 8192.0f * 360.0f;
  ptr->real_angle2 = ptr->total_angle / 18.1935f / 8192.0f * 360.0f;
  ptr->real_angle3 = ptr->total_angle / 8192.0f * 360.0f;
}

// CAN 接收回调（FIFO0 有新消息时触发）
int a, b, c;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  CAN_RxHeaderTypeDef rx_header1;
  uint8_t rx_data1[8];
  if (hcan == &hcan1) {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) !=
          HAL_OK) {
        break;
      }

      switch (rx_header.StdId) {
      case CAN_3508_M1_ID:
      case CAN_3508_M2_ID:
      case CAN_3508_M3_ID:
      case CAN_3508_M4_ID: {
        uint8_t i = (uint8_t)(rx_header.StdId - CAN_3508_M1_ID);
        if (i < 4) {
          wheel.wheel_motor[i].msg_cnt++ <= 50
              ? get_moto_offset(&wheel.wheel_motor[i], rx_data)
              : get_moto_measure(&wheel.wheel_motor[i], rx_data);
          a++;
        }
        get_moto_measure(&motor_infor, rx_data);
        break;
      }
      case CAN_3508_M5_ID:
      case CAN_3508_M6_ID: {
        uint8_t i = (uint8_t)(rx_header.StdId - CAN_3508_M5_ID);
        if (i < 2) {
          wheel.wheelO_motor[i].msg_cnt++ <= 50
              ? get_moto_offset(&wheel.wheelO_motor[i], rx_data)
              : get_moto_measure(&wheel.wheelO_motor[i], rx_data);
        }
        get_moto_measure(&motor_infor, rx_data);
        break;
      }
      case CAN_3508_M7_ID: {
        get_moto_measure(&motor_infor, rx_data);
        break;
      }
      default: {  
        break;
      }
      }
    }
    return;
  }
  if (hcan == &hcan2) {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U) {
      if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header1, rx_data1) !=
          HAL_OK) {
        break;
      }
      uint8_t i;
      switch (rx_header1.StdId) {
      case CAN2_2006_M1_ID:
      case CAN2_2006_M2_ID:
      case CAN2_2006_M3_ID:
      case CAN2_2006_M4_ID:
        i = (uint8_t)(rx_header1.StdId - CAN2_2006_M1_ID);
        if (i < 4) {
          // 修复msg_cnt判断逻辑
          if (R2006.R2006_motor_measure[i].msg_cnt++ <= 50) {
            get_moto_offset(&R2006.R2006_motor_measure[i], rx_data1);
          } else {
            get_moto_measure(&R2006.R2006_motor_measure[i], rx_data1);
          }
          get_moto_measure(&motor_infor1, rx_data1);
        }
        break;
      default:
        break;
      }
    }
  }
}

volatile uint32_t can_err_code1;
volatile uint32_t can_err_code2;

/**
 * @brief CAN错误回调函数
 * @param hcan CAN句柄指针，指向发生错误的CAN实例
 * @note 该函数在CAN总线发生错误时被调用，用于记录错误码
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  uint32_t err = HAL_CAN_GetError(hcan);  // 获取CAN错误码

  
  // 根据不同的CAN实例记录错误码
  if (hcan == &hcan1) {  // 判断是否为CAN1
    can_err_code1 = err;  // 将错误码保存到CAN1的错误变量中
  } else if (hcan == &hcan2) {  // 判断是否为CAN2
    can_err_code2 = err;  // 将错误码保存到CAN2的错误变量中
  }
}
void get_moto_offset(motor_measure_t *ptr, uint8_t data[8]) {
  ptr->ecd = (uint16_t)(data[0] << 8 | data[1]);
  ptr->offset_angle = ptr->ecd;
}

/**
 * @brief 累计角度计算（多圈角度），用于将编码器角度展开为连续角度
 */
void get_total_angle(motor_measure_t *p) {
  int res1, res2, delta;
  if (p->ecd < p->last_ecd) {           // 编码值回绕（可能过零）
    res1 = p->ecd + 8192 - p->last_ecd; // 认为正向转动的差值
    res2 = p->ecd - p->last_ecd;        // 认为反向转动的差值
  } else {                              // angle > last
    res1 = p->ecd - 8192 - p->last_ecd; // 认为反向转动的差值
    res2 = p->ecd - p->last_ecd;        // 认为正向转动的差值
  }
  // 取绝对值更小的差值作为本次转动增量（避免跨零误判）
  if (ABS(res1) < ABS(res2))
    delta = res1;
  else
    delta = res2;

  p->total_angle += delta;
  p->last_ecd = p->ecd;
}

void reset_motor_zero_angle(motor_measure_t *ptr) {
  if (ptr == NULL)
    return;

  // 将当前总角度设为offset_angle，使其成为新的零点
  ptr->offset_angle = ptr->ecd;

  // 重置圈计数和上次编码器计数（可选，防止累计误差）
  ptr->round_cnt = 0;
  ptr->last_ecd = ptr->ecd;

  // 重置total_angle为0，因为现在相对角度归零
  ptr->total_angle = 0;

  // 重新计算 real_angle，应该为0度
  ptr->real_angle1 = 0.0f;
  ptr->real_angle2 = 0.0f;

  ptr->real_angle3 = ptr->ecd * (360.0f / 8192.0f);
}

// 发送 4 个麦轮 3508 电机控制电流（CAN ID: 0x200）
void CAN_cmd_chassis_wheelM(int16_t motor1, int16_t motor2, int16_t motor3,
                            int16_t motor4) {
  uint32_t send_mail_box;
  chassis_wheel_tx_message.StdId = CAN_CHASSIS_WHEEL_ALL_ID;
  chassis_wheel_tx_message.IDE = CAN_ID_STD;
  chassis_wheel_tx_message.RTR = CAN_RTR_DATA;
  chassis_wheel_tx_message.DLC = 0x08;
  chassis_wheel_tx_message.TransmitGlobalTime = DISABLE;
  chassis_wheel_can_send_data[0] = motor1 >> 8;
  chassis_wheel_can_send_data[1] = motor1;
  chassis_wheel_can_send_data[2] = motor2 >> 8;
  chassis_wheel_can_send_data[3] = motor2;
  chassis_wheel_can_send_data[4] = motor3 >> 8;
  chassis_wheel_can_send_data[5] = motor3;
  chassis_wheel_can_send_data[6] = motor4 >> 8;
  chassis_wheel_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_WHEEL_CAN, &chassis_wheel_tx_message,
                       chassis_wheel_can_send_data, &send_mail_box);
}
static int16_t g_arm3508_can_current;

void CAN_cmd_R3508_motor(int16_t motor1) {
  g_arm3508_can_current = motor1;
}

// 发送 2 个全向轮 3508 + 机械臂 3508 电机控制电流（CAN ID: 0x1FF）
void CAN_cmd_chassis_wheelO(int16_t motor1, int16_t motor2) {
  uint32_t send_mail_box2;
  chassis_wheelO_tx_message.StdId = CAN1_last_four_ALL_ID;
  chassis_wheelO_tx_message.IDE = CAN_ID_STD;
  chassis_wheelO_tx_message.RTR = CAN_RTR_DATA;
  chassis_wheelO_tx_message.DLC = 0x08;
  chassis_wheelO_tx_message.TransmitGlobalTime = DISABLE;
  chassis_wheelO_can_send_data[0] = motor1 >> 8;
  chassis_wheelO_can_send_data[1] = motor1;
  chassis_wheelO_can_send_data[2] = motor2 >> 8;
  chassis_wheelO_can_send_data[3] = motor2;
  chassis_wheelO_can_send_data[4] = g_arm3508_can_current >> 8;
  chassis_wheelO_can_send_data[5] = g_arm3508_can_current;
  chassis_wheelO_can_send_data[6] = 0;
  chassis_wheelO_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_wheelO_CAN, &chassis_wheelO_tx_message,
                       chassis_wheelO_can_send_data, &send_mail_box2);
}
void CAN_cmd_R2006_motor(int16_t motor1, int16_t motor2, int16_t motor3,
                         int16_t motor4) {
  uint32_t send_mail_box3;
  CAN2_R2006_ID_tx_message.StdId = CAN2_R2006_ID;
  CAN2_R2006_ID_tx_message.IDE = CAN_ID_STD;
  CAN2_R2006_ID_tx_message.RTR = CAN_RTR_DATA;
  CAN2_R2006_ID_tx_message.DLC = 0x08;
  CAN2_R2006_ID_tx_message.TransmitGlobalTime = DISABLE;
  CAN2_R2006_ID_can2_send_data[0] = motor1 >> 8;
  CAN2_R2006_ID_can2_send_data[1] = motor1;
  CAN2_R2006_ID_can2_send_data[2] = motor2 >> 8;
  CAN2_R2006_ID_can2_send_data[3] = motor2;
  CAN2_R2006_ID_can2_send_data[4] = motor3 >> 8;
  CAN2_R2006_ID_can2_send_data[5] = motor3;
  CAN2_R2006_ID_can2_send_data[6] = motor4 >> 8;
  CAN2_R2006_ID_can2_send_data[7] = motor4;
  HAL_CAN_AddTxMessage(&CAN2_R2006, &CAN2_R2006_ID_tx_message,
                       CAN2_R2006_ID_can2_send_data, &send_mail_box3);
}

const motor_measure_t *get_chassis_wheel_motor_measure_point(uint8_t i) {
  return &wheel.wheel_motor[(i & 0x07)];
}

const motor_measure_t *get_chassis_wheelO_motor_measure_point(uint8_t i) {
  return &wheel.wheelO_motor[(i & 0x07)];
}

const motor_measure_t *get_R2006_motor_measure_point(uint8_t i) {
  return &R2006.R2006_motor_measure[i & 0x07];
}