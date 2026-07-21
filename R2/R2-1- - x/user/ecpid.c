
#include "ecpid.h"

#define LimitMax(input, max)   	\
{                        				\
	if (input > max)       				\
  {                      				\
		input = max;       	 				\
  }                      				\
  else if (input < -max) 				\
  {                      				\
		input = -max;        				\
  }                      				\
}

#ifndef ABS_F
#define ABS_F(x) ((x) > 0.0f ? (x) : -(x))
#endif

// 在头文件的结构体里可增加变量：fp32 I_Band; fp32 D_alpha; 和 fp32 last_Dout;


ECPidTypeDef ECPID[8];

void ECPID_init(ECPidTypeDef *ecpid,uint8_t mode,const fp32 ECPID[3],fp32 max_out,fp32 max_iout,fp32 deadband,fp32 max_err,fp32 d_filter,fp32 last_Dout)
{
	if(ecpid==0||ECPID==0)
	{
		return;
	}

	ecpid->mode=mode;
	ecpid->Kp=ECPID[0];
	ecpid->Ki=ECPID[1];
	ecpid->Kd=ECPID[2];
	ecpid->max_out=max_out;
	ecpid->max_iout=max_iout;
//    ecpid->deadband = 0.0f;
//    ecpid->max_err = 0.0f;
//    ecpid->d_filter = 1.0;
//    ecpid->last_Dout = 0.0f;
	ecpid->deadband = deadband;
    ecpid->max_err = max_err;
    ecpid->d_filter = d_filter;
    ecpid->last_Dout = last_Dout;
//*3508*//	
//    ecpid->deadband = 0.15f;    
//    ecpid->max_err = 5.0f;    
//    ecpid->d_filter = 0.8f;   
	
		//*3508//
//	ecpid->deadband = 0.4f;
//    ecpid->max_err = 8.0f;
//    ecpid->d_filter = 0.4f;
//    ecpid->last_Dout = 0.0f;
    
    
	//*6020//
//	ecpid->deadband = 0.0f;
//    ecpid->max_err = 0.0f;
//    ecpid->d_filter = 0.8f;
//    ecpid->last_Dout = 0.0f;
    
	ecpid->Dbuf[0]=ecpid->Dbuf[1]=ecpid->Dbuf[2]=0.0f;
	ecpid->error[0]=ecpid->error[1]=ecpid->error[2]=ecpid->Pout=ecpid->Iout=ecpid->Dout=ecpid->out=0.0f;
}



fp32 ECPID_Calc(ECPidTypeDef *ecpid, fp32 ref, fp32 set)
{    
    if (ecpid == 0)
    {
        return 0.0f;
    }
     
	ecpid->error[2] = ecpid->error[1];
	ecpid->error[1] = ecpid->error[0];
	ecpid->set = set;
	ecpid->fdb = ref;

	fp32 raw_err = set - ref;
	if (ABS_F(raw_err) < ecpid->deadband)
	{
		ecpid->error[0] = 0.0f;
	}
	else
	{
		ecpid->error[0] = raw_err;
	}
		
    if (ecpid->mode == ECPID_POSITION)
    {
        ecpid->Pout = ecpid->Kp * ecpid->error[0];

        // 仅使用 max_err 做积分分离阈值：max_err=0 时表示始终积分。
        if ((ecpid->max_err == 0.0f) || (ABS_F(ecpid->error[0]) < ecpid->max_err))
        {
            ecpid->Iout += ecpid->Ki * (ecpid->error[0] + ecpid->error[1]) * 0.5f;
        }
        else
        {
            // 误差过大时缓慢释放积分，减少跃迁反冲。
            ecpid->Iout *= 0.98f;
        }

        ecpid->Dbuf[2] = ecpid->Dbuf[1];
        ecpid->Dbuf[1] = ecpid->Dbuf[0];

        // 仅使用 d_filter 做微分一阶低通。
        fp32 d_alpha = ecpid->d_filter;
        if (d_alpha < 0.0f)
        {
            d_alpha = 0.0f;
        }
        else if (d_alpha > 1.0f)
        {
            d_alpha = 1.0f;
        }

        ecpid->Dbuf[0] = ecpid->error[0] - ecpid->error[1];
        {
            fp32 raw_Dout = ecpid->Kd * ecpid->Dbuf[0];
            ecpid->Dout = d_alpha * raw_Dout + (1.0f - d_alpha) * ecpid->last_Dout;
            ecpid->last_Dout = ecpid->Dout;
        }

        LimitMax(ecpid->Iout, ecpid->max_iout);
        ecpid->out = ecpid->Pout + ecpid->Iout + ecpid->Dout;
        LimitMax(ecpid->out, ecpid->max_out);
    }
    else if (ecpid->mode == ECPID_DELTA)
    {
        ecpid->Pout = ecpid->Kp * (ecpid->error[0] - ecpid->error[1]);
        ecpid->Iout = ecpid->Ki * ecpid->error[0];
        ecpid->Dbuf[2] = ecpid->Dbuf[1];
        ecpid->Dbuf[1] = ecpid->Dbuf[0];
        
        fp32 diff = ecpid->error[0] - 2.0f * ecpid->error[1] + ecpid->error[2];
        fp32 delta_alpha = (ecpid->d_filter < 0.0f) ? 0.0f : ((ecpid->d_filter > 1.0f) ? 1.0f : ecpid->d_filter);
        ecpid->Dbuf[0] = (1.0f - delta_alpha) * ecpid->Dbuf[1] + delta_alpha * diff;
        
        ecpid->Dout = ecpid->Kd * ecpid->Dbuf[0];
        ecpid->out += ecpid->Pout + ecpid->Iout + ecpid->Dout;
        LimitMax(ecpid->out, ecpid->max_out);
    }
    return ecpid->out;
}



void ECPID_clear(ECPidTypeDef *ecpid)
{
    if (ecpid == NULL)
    {
        return;
    }

    ecpid->error[0] = ecpid->error[1] = ecpid->error[2] = 0.0f;
    ecpid->Dbuf[0] = ecpid->Dbuf[1] = ecpid->Dbuf[2] = 0.0f;
    ecpid->out = ecpid->Pout = ecpid->Iout = ecpid->Dout = 0.0f;
    ecpid->last_Dout = 0.0f;
    ecpid->fdb = ecpid->set = 0.0f;
}

