#ifndef _MYTASK_H
#define _MYTASK_H
#include "freertos.h"
#include "cmsis_os.h" 
#include "task.h"
#include "CAN_receive.h"
#include "pid.h"
#include "ecpid.h"
#include "sbus.h"
#include "usart.h"
#include "string.h"
#include "dma.h"
#include "timers.h"
#include "math.h"
#include "queue.h"
#include "semphr.h"
#include "hi14_driver.h"
#include "modbus.h"

#define TIMEOUT_MS 1000  // 超时时间设为1000毫秒

#define START_TASK_PRIO		0
#define START_STK_SIZE 		128
void start_task(void *pvParameters);//开始任务

#define MOTOR_CHASSIS_TASK_PRIO	 6
#define MOTOR_CHASSIS_SIZE 		512
void motor_CHASSIS_task(void *pvParameters);//dianji

#define YAOKONG_TASK_PRIO		7
#define YAOKONG_SIZE 		256
void yaokong_task(void *pvParameters);//yaokong

#define HOW_TASK_PRIO	 7
#define HOW_SIZE 		1024
void How_task(void *pvParameters);

#define IMU_TASK_PRIO		6
#define IMU_STK_SIZE 		256
void imu_task(void *pvParameters);

extern void YS_SendCmd(void);

/* 任务句柄声明 */
extern TaskHandle_t StartTask_Handler;
extern TaskHandle_t MOTOR_CHASSISTask_Handler;
extern TaskHandle_t YAOKONGTask_Handler;
extern TaskHandle_t HOWTask_Handler;
extern TaskHandle_t IMUTask_Handler;


extern int16_t g_sbus_channels[18];
extern void sbus_loop(void);
extern void How_change(void);
extern void C_AND_GTLS_change(void);

extern volatile float g_imu_w_correction;

typedef enum
{
    CHASSIS = 0,
    order = 1,
    GET= 2
}HOW_TO_CONTROL ;

typedef enum 
{
    CHASSIS_ALL = 0,
    GTLS_MOVE = 1
}C_AND_GTLS; 

typedef enum //0x01 - 0xFF/
{
    stop = 0x00,
    grip_put = 0x01,
    three_x = 0x02,
    three_put = 0x03,
}YS_RUN_E;

#endif