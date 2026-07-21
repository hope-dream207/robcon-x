/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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
// #include "arm_math.h"
#include "struct_typedef.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

#define CHASSIS_WHEEL_CAN hcan1
#define Lift_Su_MOTOR_CAN  hcan1
#define CHASSIS_STEERING_CAN hcan2
#define CAN2_GT hcan2

#define OMIN_CONV (60.0f / (2.0f * PI)) // 角速度转换为轮子转速的转换系数
#define SMALLTOP_CIRCLE   45.0f // 小陀螺角度，默认45度(宽长近似相等); 可按atan(宽/长)调整
#define DEG2R(x) ((x)*PI /180.0f)
#define R2DEG(x) ((x)*180.0f /PI)
#define ABS(x)	((x>0) ? (x) : (-x) )
/* CAN send and receive ID */
typedef enum
{  //CAN1
    CAN1_ALLF_ID = 0x200,
    CAN1_CHASSIS_3508_1ID = 0x201,
    CAN1_CHASSIS_3508_2ID = 0x202,
    CAN1_CHASSIS_3508_3ID = 0x203,
    CAN1_CHASSIS_3508_4ID = 0x204,

    CAN1_LIFT_3508_5ID = 0x205,
    CAN1_LIFT_3508_6ID = 0x206,
    CAN1_St_2006_7ID = 0x207,
    CAN1_M8ID = 0x208,
    CAN_LIFT_su_MOTOR_ALL_ID = 0x1FF,
    //CAN2
    CAN2_ALLF_ID = 0x200,
    CAN2_M1_ID = 0x201,
    CAN2_M2_ID = 0x202,
    CAN2_M3_ID = 0x203,
    CAN2_M4_ID=  0x204,

    CAN2_6020_M1_ID = 0x205,
    CAN2_2006_M2_ID = 0x206,
    CAN2_M7_ID = 0x207,
    CAN2_M8_ID=  0x208,
    CAN_GIS_ALL_ID = 0x1FF,
} can_msg_id_e;

 //CAN读取数据结构体
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
   float real_angle1;   // 实际角度（单位：度）2006
   float real_angle2;   // 实际角度（单位：度）3508
   int32_t total_angle1;   // 累计角度（多圈）6020
   uint16_t offset_angle1; // 上电时的角度零偏 6020
   float real_angle3;   // 实际角度（单位：度）6020
} motor_measure_t;


  extern void get_total_angle(motor_measure_t *p);
  extern void reset_motor_zero_angle(motor_measure_t *ptr);
  extern void get_moto_offset(motor_measure_t *ptr, uint8_t data[8]);
  extern void get_moto_measure(motor_measure_t *ptr, uint8_t data[8]);

  extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
  extern void CAN_cmd_Lift_su_motor(int16_t motor1, int16_t motor2, int16_t motor3);
  extern void CAN_cmd_GT_motor(int16_t motor1, int16_t motor2);  

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_wheel_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_Lift_su_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_GT_motor_measure_point(uint8_t i);
extern const motor_measure_t *get_motor_8_measure_point(void);



#endif
