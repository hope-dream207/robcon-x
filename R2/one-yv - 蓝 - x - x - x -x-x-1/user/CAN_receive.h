/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             包含 CAN 中断接收电机反馈数据，以及 CAN 发送电机电流控制指令的接口。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H


#include "struct_typedef.h"
#include "pid.h"
#include "ecpid.h"


#define CHASSIS_WHEEL_CAN hcan1 // 底盘麦轮（4个电机）CAN 句柄，对应 CAN_CHASSIS_WHEEL_ALL_ID = 0x200
#define CHASSIS_wheelO_CAN hcan1 // 全向轮（2个电机）CAN 句柄，对应 CAN1_last_four_ALL_ID = 0x1FF
#define CAN1_R2006 hcan1
#define CAN2_R2006 hcan2
#define CAN1_R3508 hcan1


#define SMALLTOP_CIRCLE   45 // 小陀螺/小转圈角度（度），用于三角函数相关计算
#define PI 3.14159265358979323846f
#define DEG2R(x) ((x)*PI /180.0f)
#define R2DEG(x) ((x)*180.0f /PI)
#define ABS(x)	((x>0) ? (x) : (-x) )
#define CONV (60.0f / (2.0f * PI)) // 角速度转换为轮子转速的转换系数
/* CAN send and receive ID */
typedef enum
{  /*================CAN1================*/
    CAN_CHASSIS_WHEEL_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

	  CAN1_last_four_ALL_ID = 0x1FF,
	  CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_3508_M7_ID = 0x207,
    CAN_3508_M8_ID = 0x208,
	
	
	CAN2_R2006_ID = 0x200,
	CAN2_2006_M1_ID = 0x201,
  CAN2_2006_M2_ID = 0x202,
  CAN2_2006_M3_ID = 0x203,
  CAN2_2006_M4_ID = 0x204,
  //不用
  CAN2_G_ID = 0x1FF, 
  CAN2_3508_M5_ID = 0x205,
  CAN2_3508_M6_ID=  0x206,
  CAN2_2006_M7_ID=  0x207,

} can_msg_id_e;

//rm motor datad 
typedef struct
{
  uint16_t ecd;          // 机械角度编码值
  int16_t speed_rpm;     // 转速（rpm）
  int16_t current_raw;   // 原始电流值（CAN 字节解析，由 calc_moto_float_values 转换为浮点）
  fp32 given_current;    // 电流反馈/电流值（原始单位以电机协议为准）
  uint8_t temperate;     // 温度
  int16_t last_ecd;      // 上一次机械角度编码值
  int16_t round_cnt;     // 圈数计数（过零累计）
  uint32_t msg_cnt;      // 接收报文计数
  int32_t total_angle;   // 累计角度（多圈）
  uint16_t offset_angle; // 上电时的角度零偏
	
  float real_angle1;
  float real_angle2;
  float real_angle3;
} motor_measure_t;


/**
  * @brief          发送 ID=0x700 的 CAN 帧，用于设置 3508 电机快速分配 ID
  * @param[in]      none
  * @retval         none
  */


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送底盘 3508 电机控制电流（0x201, 0x202, 0x203, 0x204）
  * @param[in]      motor1: (0x201) 3508 电机控制电流，范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508 电机控制电流，范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508 电机控制电流，范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508 电机控制电流，范围 [-16384,16384]
  * @retval         none
  */
  extern void get_total_angle(motor_measure_t *p);
  extern void get_moto_offset(motor_measure_t *ptr, uint8_t data[8]);
  extern void get_moto_measure(motor_measure_t *ptr, uint8_t data[8]);
/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          获取底盘 3508 电机反馈数据指针
  * @param[in]      i: 电机编号，范围 [0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_wheel_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_chassis_wheelO_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_R2006_motor_measure_point(uint8_t i);
extern void CAN_cmd_R2006_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_R3508_motor(int16_t motor1);
extern void CAN_cmd_chassis_wheelM(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis_wheelO(int16_t motor1, int16_t motor2);
#endif