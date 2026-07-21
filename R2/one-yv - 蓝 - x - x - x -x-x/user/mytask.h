 #ifndef _MYTASK_H
#define _MYTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "CAN_receive.h"
#include "pid.h"
#include "ecpid.h"
#include "l1_laser.h"
#include "usart.h"
#include "dma.h"
#include <math.h>
#include "usartx.h"

#define TIMEOUT_MS 1000  // 超时时间设为1000毫秒

#define START_TASK_PRIO		8
#define START_STK_SIZE 		128
void start_task(void *pvParameters);//开始任务

#define M8010_TASK_PRIO  7
#define M8010_STK_SIZE   256
void M8010_CommTask(void *pvParameters);

#define CHASSIS_TASK_PRIO		6
#define CHASSIS_STK_SIZE 		256
void chassis_task(void *pvParameters);//底盘任务

#define IMU_TASK_PRIO		6
#define IMU_STK_SIZE 		256
void imu_task(void *pvParameters);//IMU任务

#define L1_TASK_PRIO		6
#define L1_STK_SIZE 		256
void l1_task(void *pvParameters);

#define HOW_TASK_PRIO	 7
#define HOW_SIZE 		1024
void How_task(void *pvParameters);

#define ROS_TASK_PRIO		7
#define ROS_SIZE 		1536
void ros_task(void *pvParameters);


extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t ChassisTask_Handler; 
extern TaskHandle_t IMUTask_Handler;
extern TaskHandle_t L1Task_Handler;
extern TaskHandle_t HOWTask_Handler;
extern TaskHandle_t M8010Task_Handler;
extern TaskHandle_t ROSTask_Handler;


typedef enum
{
    ONE = 0,
    TWO = 1,
    THREE = 2
}HOW_TO_CONTROL ;

typedef enum
{
    grip_close = 0, // 气抓闭合
    grip_put = 1,  // 气抓打开
}GRIP_CONTROL;

typedef enum
{
    lift_down = 0, // 气抓下降
    lift_up = 1    // 气抓上升
}LIFT_CONTROL;

typedef enum
{
    rotate_stop = 0, // 气抓停止
    rotate_turn = 1  // 气抓旋转
}ROTATE_CONTROL;

typedef enum
{
    stop = 0,
    getx_L = 1,
    getx_H = 2,//剩下是3区的
    get_put = 3,//3区的取
    arm_put = 4,//3区的放
    stop_3 = 5

}ARM_HOW;

typedef enum
{
    up = 0,
    down = 1,
    arm_finish = 2
}ARM_updown;


typedef enum {
    open   = 0,
    first  = 1,
    second = 2,
    third  = 3,
    fourth = 4,
    fifth  = 5,
    sixth  = 6,
    seventh = 7,
    eighth = 8,
    ninth = 9,
} Order;

typedef enum //0x01 - 0xFF/
{
    stop_ = 0x00,
    grip_put_ = 0x01,
    three_x = 0x02,
    three_put = 0x03,
}YS_RUN_E;

/* 里程计数据 (局部坐标系, 米/度, 纯编码器) */
typedef struct {
    float x;        /* 局部系 X 坐标, m */
    float y;        /* 局部系 Y 坐标, m */
    float yaw;      /* 偏航角, deg (IMU 透传) */
    float yaw_rad;  /* 偏航角, rad (避免重复计算) */
} Odom_Data;


extern ARM_HOW ARM_how;
extern uint8_t cylinder_allow;
extern HOW_TO_CONTROL how;
extern GRIP_CONTROL grip_run;
extern LIFT_CONTROL lift_run;
extern ROTATE_CONTROL rotate_run;
extern Odom_Data odom;


#endif