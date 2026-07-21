#ifndef __ARM_H__
#define __ARM_H__

void arm_final(void);
void arm_change(void);
void arm_control_motors(void);

typedef struct
{
    float o1;
    float o2;
    float o3;
    float set1;
    float set2;
    float set3;
} ARM_TO;

extern ARM_TO ARM_to;

#endif