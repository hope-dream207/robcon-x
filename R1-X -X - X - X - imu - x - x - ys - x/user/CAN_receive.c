#include "CAN_receive.h"
#include "Lift_sucker.h"
#include "Omin.h"
#include "grip_turnd.h"
#include "main.h"
#include "stm32f4xx_hal_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern int16_t g_sbus_channels[18];
extern Chassis_TypeDef chassis;
extern Lift_su_type Lift_su;
extern GT_TypeDef GT;

motor_measure_t motor_infor;
motor_measure_t motor_infor1;

static CAN_TxHeaderTypeDef chassis_wheel_tx_message;
static uint8_t chassis_wheel_can_send_data[8];

static CAN_TxHeaderTypeDef Lift_su_motor_tx_message;
static uint8_t Lift_su_motor_can_send_data[8];

static CAN_TxHeaderTypeDef CAN2_GT_ID_tx_message;
static uint8_t GT_ID_can2_send_data[8];

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
  ptr->total_angle1 = ptr->round_cnt * 8192 + ptr->ecd ;
  ptr->real_angle1 = ptr->total_angle / 36.1935f / 8192.0f * 360.0f;
  ptr->real_angle2 = ptr->total_angle / 18.1935f / 8192.0f * 360.0f;
  ptr->real_angle3 = ptr->total_angle1 / 8192.0f * 360.0f;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  CAN_RxHeaderTypeDef rx_header1;
  uint8_t rx_data[8];
  uint8_t rx_data1[8];

  if (hcan->Instance == CAN1) {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        break;
      }
      switch (rx_header.StdId) {
      case CAN1_CHASSIS_3508_1ID:
      case CAN1_CHASSIS_3508_2ID:
      case CAN1_CHASSIS_3508_3ID:
      case CAN1_CHASSIS_3508_4ID: {
        uint8_t i = (uint8_t)(rx_header.StdId - CAN1_CHASSIS_3508_1ID);
        if (i < 4) {
          if (chassis.Omni_motor_measure[i].msg_cnt <= 50) {
            chassis.Omni_motor_measure[i].msg_cnt++;
            get_moto_offset(&chassis.Omni_motor_measure[i], rx_data);
          } else {
            get_moto_measure(&chassis.Omni_motor_measure[i], rx_data);
          }
          get_moto_measure(&motor_infor, rx_data);
        }
        break;
      }
      case CAN1_LIFT_3508_5ID:
      case CAN1_LIFT_3508_6ID:
      case CAN1_St_2006_7ID: {
        uint8_t i = (uint8_t)(rx_header.StdId - CAN1_LIFT_3508_5ID);
        if (i < 3) {
          if (Lift_su.lift_su_motor_measure[i].msg_cnt <= 50) {
            Lift_su.lift_su_motor_measure[i].msg_cnt++;
            get_moto_offset(&Lift_su.lift_su_motor_measure[i], rx_data);
          } else {
            get_moto_measure(&Lift_su.lift_su_motor_measure[i], rx_data);
          }
          get_moto_measure(&motor_infor, rx_data);
        }
        break;
      }
      default:
        break;
      }
    }
    return;
  }

  if (hcan->Instance == CAN2) {
    while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0U) {
      if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header1, rx_data1) != HAL_OK) {
        break;
      }
      switch (rx_header1.StdId) {
      case CAN2_6020_M1_ID:
      case CAN2_2006_M2_ID: {
        uint8_t i = (uint8_t)(rx_header1.StdId - CAN2_6020_M1_ID);
        if (i < 2) {
          if (GT.GT_motor_measure[i].msg_cnt <= 50) {
            GT.GT_motor_measure[i].msg_cnt++;
            get_moto_offset(&GT.GT_motor_measure[i], rx_data1);
          } else {
            get_moto_measure(&GT.GT_motor_measure[i], rx_data1);
          }
          get_moto_measure(&motor_infor1, rx_data1);
        }
        break;
      }
      default:
        break;
      }
    }
    return;
  }
}
volatile uint32_t can_err_code1;
volatile uint32_t can_err_code2;
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
 *@bref 将当前角度清零作为初始值，之后的角度都是作为相对值处理
 */
void get_total_angle(motor_measure_t *p) {
  int res1, res2, delta;
  if (p->ecd < p->last_ecd) {
    res1 = p->ecd + 8192 - p->last_ecd;
    res2 = p->ecd - p->last_ecd;
  } else {
    res1 = p->ecd - 8192 - p->last_ecd;
    res2 = p->ecd - p->last_ecd;
  }
  
  if (ABS(res1) < ABS(res2)) {
    delta = res1;
  } else {
    delta = res2;
  }

  p->total_angle += delta;
  p->last_ecd = p->ecd;
}

/**
 * @brief  将当前编码器角度设为零点（重置偏移），使当前角度变为0度
 * @param  ptr  电机状态结构体指针
 */
void reset_motor_zero_angle(motor_measure_t *ptr) {
  if (ptr == NULL) {
    return;
  }

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

/**
 * @brief          CAN发送电机控制指令
 * @param[in]      motor1: 电机1的控制值，motor2: 电机2的控制值，motor3:
 * 电机3的控制值，motor4: 电机4的控制值
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis(int16_t moto1, int16_t moto2, int16_t moto3, int16_t moto4) {
  uint32_t tx_mailbox;
  chassis_wheel_tx_message.StdId = CAN1_ALLF_ID;
  chassis_wheel_tx_message.IDE = CAN_ID_STD;
  chassis_wheel_tx_message.RTR = CAN_RTR_DATA;
  chassis_wheel_tx_message.DLC = 0x08;
  chassis_wheel_tx_message.TransmitGlobalTime = DISABLE;
  
  chassis_wheel_can_send_data[0] = (moto1 >> 8) & 0xFF;
  chassis_wheel_can_send_data[1] = moto1 & 0xFF;
  chassis_wheel_can_send_data[2] = (moto2 >> 8) & 0xFF;
  chassis_wheel_can_send_data[3] = moto2 & 0xFF;
  chassis_wheel_can_send_data[4] = (moto3 >> 8) & 0xFF;
  chassis_wheel_can_send_data[5] = moto3 & 0xFF;
  chassis_wheel_can_send_data[6] = (moto4 >> 8) & 0xFF;
  chassis_wheel_can_send_data[7] = moto4 & 0xFF;
  
  (void)HAL_CAN_AddTxMessage(&CHASSIS_WHEEL_CAN, &chassis_wheel_tx_message, chassis_wheel_can_send_data, &tx_mailbox);
}

void CAN_cmd_Lift_su_motor(int16_t motor1, int16_t motor2, int16_t motor3) {
  uint32_t send_mail_box1;
  Lift_su_motor_tx_message.StdId = CAN_LIFT_su_MOTOR_ALL_ID;
  Lift_su_motor_tx_message.IDE = CAN_ID_STD;
  Lift_su_motor_tx_message.RTR = CAN_RTR_DATA;
  Lift_su_motor_tx_message.DLC = 0x08;
  Lift_su_motor_tx_message.TransmitGlobalTime = DISABLE;
  
  Lift_su_motor_can_send_data[0] = motor1 >> 8;
  Lift_su_motor_can_send_data[1] = motor1;
  Lift_su_motor_can_send_data[2] = motor2 >> 8;
  Lift_su_motor_can_send_data[3] = motor2;
  Lift_su_motor_can_send_data[4] = motor3 >> 8;
  Lift_su_motor_can_send_data[5] = motor3;
  Lift_su_motor_can_send_data[6] = 0;
  Lift_su_motor_can_send_data[7] = 0;
  
  (void)HAL_CAN_AddTxMessage(&Lift_Su_MOTOR_CAN, &Lift_su_motor_tx_message, Lift_su_motor_can_send_data, &send_mail_box1);
}

void CAN_cmd_GT_motor(int16_t motor1, int16_t motor2) {
  uint32_t send_mail_box2;
  CAN2_GT_ID_tx_message.StdId = CAN_GIS_ALL_ID;
  CAN2_GT_ID_tx_message.IDE = CAN_ID_STD;
  CAN2_GT_ID_tx_message.RTR = CAN_RTR_DATA;
  CAN2_GT_ID_tx_message.DLC = 0x08;
  CAN2_GT_ID_tx_message.TransmitGlobalTime = DISABLE;
  
  GT_ID_can2_send_data[0] = motor1 >> 8;
  GT_ID_can2_send_data[1] = motor1;
  GT_ID_can2_send_data[2] = motor2 >> 8;
  GT_ID_can2_send_data[3] = motor2;
  GT_ID_can2_send_data[4] = 0;
  GT_ID_can2_send_data[5] = 0;
  GT_ID_can2_send_data[6] = 0;
  GT_ID_can2_send_data[7] = 0;

  (void)HAL_CAN_AddTxMessage(&CAN2_GT, &CAN2_GT_ID_tx_message, GT_ID_can2_send_data, &send_mail_box2);
}

const motor_measure_t *get_chassis_wheel_motor_measure_point(uint8_t i) {
  return &chassis.Omni_motor_measure[(i & 0x07)];
}

const motor_measure_t *get_Lift_su_motor_measure_point(uint8_t i) {
  return &Lift_su.lift_su_motor_measure[(i & 0x07)];
}

const motor_measure_t *get_GT_motor_measure_point(uint8_t i) {
  return &GT.GT_motor_measure[(i % 0x07)];
}