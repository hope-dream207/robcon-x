#include "control_Cylinder.h"
#include "arm.h"
#include "mytask.h"
#include "FreeRTOS.h"
#include "stm32f429xx.h"
#include "task.h"
#include "stm32f4xx_hal_gpio.h"
#include "usartx.h"
#include "mytask.h"

extern GRIP_CONTROL grip_run;
extern LIFT_CONTROL lift_run;
extern ROTATE_CONTROL rotate_run;
extern ARM_HOW arm_how;
extern HOW_TO_CONTROL how;
extern YS_RUN_E y_run;


volatile GPIO_PinState g_gpioa11_level = GPIO_PIN_RESET;
void Cylinder_Init(void) // 气缸初始化
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,
                    GPIO_PIN_RESET); // 爪子关闭
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,
                    GPIO_PIN_RESET); // 爪子上升	
  g_gpioa11_level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                    GPIO_PIN_RESET); // arm吸取
}

//**************************爪子**************************//
void Cylinder_Control(void) {
  if (grip_run == grip_put) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET); // 爪子打开
  } else if (grip_run == grip_close) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // 爪子关闭
  }
  if (lift_run == lift_up) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // 爪子上升
  } else if (lift_run == lift_down) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // 爪子下降
  }
  g_gpioa11_level = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
}

//**************************arm**************************写3区了//
void Cylinder_Controlx(void) // 改为 void，因为你在函数内部直接读取遥控器数据，不需要传参了
{
    if (arm_how == getx_L || arm_how == getx_H || arm_how == get_put) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // arm吸取
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // arm吸取断开
   }
}

void Cylinder_Controly(void)//存储
{   if (y_run == three_put) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // 断开
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // 吸取
    }
}