#ifndef CONTORL_H
#define CONTORL_H

#include "stm32f4xx_hal.h"
#include "freertos.h"
#include "queue.h"
#include "sbus.h"

void Cylinder_Init(void);
void Cylinder_Control(void);
void Cylinder_Controlx(void);
void Cylinder_ForceOff(uint8_t pin_index);

#endif