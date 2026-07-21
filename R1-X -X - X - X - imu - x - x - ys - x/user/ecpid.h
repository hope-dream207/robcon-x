#ifndef _ecpid_h
#define _ecpid_h

#include "main.h"
#include "stdint.h"

typedef float fp32;

enum ECPID_MODE {
  ECPID_POSITION = 0,
  ECPID_DELTA
};


typedef struct//pid
{
  uint8_t mode;
  fp32 Kp;
  fp32 Ki;
  fp32 Kd;

  fp32 max_out;  
  fp32 max_iout; 

  fp32 set;
  fp32 fdb;

  fp32 out;
  fp32 Pout;
  fp32 Iout;
  fp32 Dout;
  fp32 Dbuf[3];  
  fp32 error[3]; 

  fp32 deadband;
  fp32 max_err;
  fp32 d_filter;
  fp32 last_Dout;
    // 增加变量：积分分离阈值、微分滤波系数、上次微分输出

}ECPidTypeDef;



fp32 ECPID_Calc(ECPidTypeDef *ecpid, fp32 ref, fp32 set);
void ECPID_init(ECPidTypeDef *ecpid, uint8_t mode, const fp32 ECPID[3], fp32 max_out, fp32 max_iout, fp32 deadband, fp32 max_err, fp32 d_filter, fp32 last_Dout);
void ECPID_clear(ECPidTypeDef *ecpid);



#endif