#include "arm.h"
#include "mytask.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "unitree.h"
#include "gom_protocol.h"

extern ARM_HOW arm_how;
ARM_TO ARM_to = {1.23f, 1.36f, -3.7f, 1.23f, 1.36f, -3.7f};
extern MotorData_t data[MOTOR_NUM];

#define ARM_O1_MIN (-13.0f)  // 关节0(肩部)最小角度 rad
#define ARM_O1_MAX (1.22f)   // 关节0(肩部)最大角度 rad
#define ARM_O2_MIN (-1.29f)  // 关节1最小角度 rad
#define ARM_O2_MAX (12.0f)   // 关节1最大角度 rad
#define ARM_O3_MIN (-19.0f)  // 关节2最小角度 rad
#define ARM_O3_MAX (3.8f)   // 关节2最大角度 rad

//控制指令更新
void arm_change()
	{//限幅0:1.22,-13  1.1.29，12 2.-13,3      
    switch (arm_how)
    {//关节侧弧度，Unitree_SetMotor内部会 ×6.33 转为电机侧
        case stop://(1.2,-13.8)(1.3,16.14)(10.8,-7.5)
           ARM_to.o1 = 1.23f;
            ARM_to.o2 = 1.36f;
            ARM_to.o3 = -3.7f;
            break;
          case getx_L:
            ARM_to.o1 = 1.23f;
            ARM_to.o2 = 1.36f;
            ARM_to.o3 = -3.7f;
            break;
        case getx_H:
            ARM_to.o1 = 1.23f;
            ARM_to.o2 = 1.36f;
            ARM_to.o3 = -3.7f;
            break;
        case get_put://-11.00;13.5;-7.425（换点前）
            ARM_to.o1 =  -9.10f;
            ARM_to.o2 =   9.10f;
            ARM_to.o3 =  -17.00f;
            break;
        case arm_put://-8.37 10.54 -11.8//-8.0,10.10//-10,12,-13
            // ARM_to.o1 = -8.0;-8-1.23=-7.77，11.0f-1.36f=9.78f,-13.7f-3.7f=-11.28f
            // ARM_to.o1 =  -7.8f-0.5f;
            // ARM_to.o2 =   9.78f+1.30+0.5f;
            // ARM_to.o3 =  -16.00f;
            ARM_to.o1 =  -9.10f;
            ARM_to.o2 =   9.10f;
            ARM_to.o3 =  -18.00f;
            break;//-7.9,9.78,-11.28,u
        case stop_3://1.19;1.29;5.74   //换0点后，1.23  1.36   -3.7
            // ARM_to.o1 = 1.23f;
            // ARM_to.o1 = 6.26f;
            // ARM_to.o2 = 1.36f;
            // ARM_to.o3 = -3.7f;-3.7-4.43=-8.13
            ARM_to.o1 = 0.0f;
            ARM_to.o2 = 0.0f+1.30f;  
            ARM_to.o3 = -8.13f;
            break;
        default:
            break;
    }//-12.5+3.7=-8.8-8.13=-16.93
}
//1:-9.3,2:-13.8
float start_o1 = 1.23f;
float start_o2 = 1.36f;
float start_o3 = -3.7f;
float target_o1 = 1.23f;
float target_o2 = 1.36f;
float target_o3 = -3.7f;
TickType_t joint_start_tick = 0;
uint8_t joint_interp_active = 0;

//最终解算
/**
 * @brief 机械臂最终控制函数
 * 该函数实现机械臂的正逆运动学融合解算，并规划平滑的运动轨迹
 */
void arm_final()
{
    arm_change();

    {
        float new_target_o1 = ARM_to.o1;
        float new_target_o2 = ARM_to.o2;
        float new_target_o3 = ARM_to.o3;
        const float target_eps = 1e-3f;
        uint8_t target_changed = fabsf(new_target_o1 - target_o1) > target_eps|| fabsf(new_target_o2 - target_o2) > target_eps|| fabsf(new_target_o3 - target_o3) > target_eps;
        if (target_changed)
        {
            target_o1 = new_target_o1;
            target_o2 = new_target_o2;
            target_o3 = new_target_o3;
            start_o1 = ARM_to.set1;
            start_o2 = ARM_to.set2;
            start_o3 = ARM_to.set3;
            joint_start_tick = xTaskGetTickCount();
            joint_interp_active = 1;
        }
        if (joint_interp_active)
        {
            const float transition_ms = 1500.0f;// 过渡时间（毫秒）
            const float transition_ms_set1 = 1500.0f;// 过渡时间（毫秒）大轴
            float elapsed_ms = (float)(xTaskGetTickCount() - joint_start_tick) * 1000.0f / (float)configTICK_RATE_HZ;
            float t_all = elapsed_ms / transition_ms;
            float t_set1 = elapsed_ms / transition_ms_set1;
            if (t_all >= 1.0f)
            {
                t_all = 1.0f;
                joint_interp_active = 0;
            }
            if (t_set1 >= 1.0f)
            {
                t_set1 = 1.0f;
            }
            float t_set1_smooth = (1.0f - cosf(t_set1 * PI)) * 0.5f;//0.5是归一化
            float t_all_smooth  = (1.0f - cosf(t_all  * PI)) * 0.5f;
            ARM_to.set1 = start_o1 + (target_o1 - start_o1) * t_set1_smooth;
            ARM_to.set2 = start_o2 + (target_o2 - start_o2) * t_all_smooth;
            ARM_to.set3 = start_o3 + (target_o3 - start_o3) * t_all_smooth;
        }
        else
        {
            const float alpha = 0.3f;
            ARM_to.set1 += (target_o1 - ARM_to.set1) * alpha;
            ARM_to.set2 += (target_o2 - ARM_to.set2) * alpha;
            ARM_to.set3 += (target_o3 - ARM_to.set3) * alpha;
        }

        ARM_to.set1 = fmaxf(ARM_O1_MIN, fminf(ARM_O1_MAX, ARM_to.set1));
        ARM_to.set2 = fmaxf(ARM_O2_MIN, fminf(ARM_O2_MAX, ARM_to.set2));
        ARM_to.set3 = fmaxf(ARM_O3_MIN, fminf(ARM_O3_MAX, ARM_to.set3));
    }
}

extern void Unitree_SetMotor(uint8_t motor_id, float q, float dq, float tau, float kp, float kd);

void arm_control_motors()
 {
//     static float prev_set1 = 1.23f;
    // static float prev_set1 = 6.26f;
    // static float prev_set2 = 1.36f;
    // static float prev_set3 = -3.7f;
    // float delta0 = ARM_to.set1 - prev_set1;
    // float delta1 = ARM_to.set2 - prev_set2;
    // float delta2 = ARM_to.set3 - prev_set3;
    // const float ff_thresh = 0.0005f;
    // float sign0 = (delta0 >  ff_thresh) ?  1.0f : ((delta0 < -ff_thresh) ? -1.0f : 0.0f);
    // float sign1 = (delta1 >  ff_thresh) ?  1.0f : ((delta1 < -ff_thresh) ? -1.0f : 0.0f);
    // float sign2 = (delta2 >  ff_thresh) ?  1.0f : ((delta2 < -ff_thresh) ? -1.0f : 0.0f);
    // prev_set1 = ARM_to.set1;
    // prev_set2 = ARM_to.set2;
    // prev_set3 = ARM_to.set3;
    Unitree_SetMotor(0, ARM_to.set1, 0.0f, 0, 2.0f, 0.1f);
    Unitree_SetMotor(1, ARM_to.set2, 0.0f, 0, 2.0f, 0.1f);
    Unitree_SetMotor(2, ARM_to.set3, 0.0f, 0, 1.0f, 0.2f);
   //    Unitree_SetMotor(2, ARM_to.set3, 0.0f, 0, 0.2f, 0.2f);
    //   Unitree_SetMotor(0, ARM_to.set1,  0.0f,  0, 0.0f, 0.0f);
    //   Unitree_SetMotor(1, -ARM_to.set1, 0.0f, 0, 0.0f, 0.0f);
    //   Unitree_SetMotor(2, -ARM_to.set2, 0.0f,  0, 0.0f, 0.0f);
    // // Unitree_SetMotor(0, ARM_to.set1,  0.0f,  0, 0.35f, 0.0f);
    // Unitree_SetMotor(1, ARM_to.set2, 0.0f, 0, 0.35f, 0.0f);
    // Unitree_SetMotor(2, ARM_to.set3, 0.0f,  0, 0.2f, 0.0f);
}