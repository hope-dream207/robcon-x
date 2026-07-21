#include "pid.h"
#include "ecpid.h"
#include "main.h"

pid_type_def PID[8];

//M3508：上限 20000 rpm
//M2006：上限 12000 rpm
//GM6020：上限 2500 rpm

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }		

#ifndef ABS_F
#define ABS_F(x) ((x) > 0.0f ? (x) : -(x))
#endif
		
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout,fp32 deadband,fp32 max_err,fp32 d_filter,fp32 Kf)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];///系统开始围绕目标值轻微震荡，然后将 Kp 乘以 0.6~0.7 作为基础值
    pid->Ki = PID[1];///Ki 太大，系统会低频大幅度震荡（像波浪一样
    pid->Kd = PID[2];///阻尼/刹车

    // 输出限幅
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    
    // 初始化优化算法参数，目标是电机之间转速的比较
    // 滤波系数f -> 基础参数 (P->I->D) -> 优化参数 (死区/积分分离)

    pid->deadband = deadband;// 死区范围，单位与输入相同，只剩I项在死区内工作，消除到达目标后的“滋滋”声或微小抖动
    pid->max_err = max_err; // 积分分离，最大误差，超过此值不积分（调完I再加）
    pid->d_filter = d_filter;// D项滤波系数，范围0-1，值越小滤波效果越明显

    pid->Kf = Kf;// 目标速度前馈增益
    // 调试方法：先把 Kf 调大到能让电机在目标速度附近运行，然后再根据需要适当降低 Kf 来平衡响应速度和稳定性。
    //f: 如果编码器/传感器信号噪声很大（波形毛刺多）：保持 0.1 ~ 0.2。如果信号很干净，且觉得 D 项反应太慢：可以加大到 0.5 ~ 0.8
    
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}   


fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
	
    if (pid == NULL)
    {
        return 0.0f;
    }

				pid->error[2] = pid->error[1];
				pid->error[1] = pid->error[0];
			
				pid->set = set;
				pid->fdb = ref;
                
                fp32 raw_err = set - ref;
                if (ABS_F(raw_err) < pid->deadband) {
                    pid->error[0] = 0.0f; 
                } else {
                    pid->error[0] = raw_err;
                }
		
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        
        // 积分分离 + 动态抗饱和(Anti-Windup)
        // 只有未达到最大输出，或者误差方向与饱和方向相反时，才继续积分
        uint8_t windup_blocked = 0;
        if ((pid->out >= pid->max_out && pid->error[0] > 0.0f) || 
            (pid->out <= -pid->max_out && pid->error[0] < 0.0f)) {
            windup_blocked = 1;
        }

        if ((ABS_F(pid->error[0]) < pid->max_err) && !windup_blocked) {
            pid->Iout += pid->Ki * (pid->error[0] + pid->error[1]) / 2.0f;
        }
        LimitMax(pid->Iout, pid->max_iout);
        
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        
        fp32 diff = pid->error[0] - pid->error[1];
        pid->Dbuf[0] = (1.0f - pid->d_filter) * pid->Dbuf[1] + pid->d_filter * diff;
        
        pid->Dout = pid->Kd * pid->Dbuf[0];
        // 加入目标速度前馈 (Feedforward)
        fp32 Ffout = pid->Kf * pid->set;
        
        pid->out = pid->Pout + pid->Iout + pid->Dout + Ffout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
			
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        
        fp32 diff = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
        pid->Dbuf[0] = (1.0f - pid->d_filter) * pid->Dbuf[1] + pid->d_filter * diff;
        
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

