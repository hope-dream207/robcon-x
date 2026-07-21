#include "laser.h"
uint8_t SEND_Rx[4];

//float pid_laser[3]={1,0.001,0.5};
float pid_laser[3]={1.2,0,0};
float pid_laser_1[3]={1.2,0,0};

uint8_t laser_on(void)
{
	  SEND_Rx[0]=0x80;
	  SEND_Rx[1]=0x06;
	  SEND_Rx[2]=0x03;
	  SEND_Rx[3]=0x77;
	//HAL_UART_Transmit(&huart3,SEND_Rx,4,100);
	HAL_UART_Transmit(&huart4,SEND_Rx,4,100);
	
  //if(HAL_UART_Transmit(&huart3,SEND_Rx,4,100)!=HAL_OK&&HAL_UART_Transmit(&huart4,SEND_Rx,4,100)!=HAL_OK)
	if(HAL_UART_Transmit(&huart4,SEND_Rx,4,100)!=HAL_OK)
	   return HAL_ERROR;
  else
     return HAL_OK;		
}

uint8_t USART_RX_BUF[128];
uint8_t USART_RX_STA=0;
uint8_t USART_RX_BUF_1[128];
uint8_t USART_RX_STA_1=0;
int value[2]={0};
uint8_t rx_buff;
uint8_t rx_buff_1;

void Handler(void)                    //串口1中断服务程序
{
    uint8_t Res;    
		Res =rx_buff;    //读取接收到的数据
		USART_RX_BUF[USART_RX_STA]=Res ;
		if(USART_RX_BUF[0]==0x80)
		{
				USART_RX_STA++;
				if( USART_RX_STA>2 )
				{
						if((USART_RX_BUF[1]==0x06) && (USART_RX_BUF[2]==0x83))
						{
								if(USART_RX_STA>=USART_REC_LEN) 
								{    
										USART_RX_STA=0;    
										if((USART_RX_BUF[3]!='E')&&(USART_RX_BUF[3]<0x34)&&(USART_RX_BUF[4]!='R')&&(USART_RX_BUF[5]!='R')
											&&(USART_RX_BUF[10]==(uint8_t)(~(0x80+0x06+0x83+USART_RX_BUF[3]+USART_RX_BUF[4]+USART_RX_BUF[5]+0x2E+USART_RX_BUF[7]+USART_RX_BUF[8]+USART_RX_BUF[9])+1))) 
										{                
												value[0] = (USART_RX_BUF[4]-0x30)*10000+(USART_RX_BUF[5]-0x30)*1000 + (USART_RX_BUF[7]-0x30)*100 + (USART_RX_BUF[8]-0x30)*10 + (USART_RX_BUF[9]-0x30);                 
												
										}    
								}
						}
						else USART_RX_STA=0;        
				}            
		} 
		else USART_RX_STA=0;            
}

void Handler_1(void)                    //串口1中断服务程序
{
    uint8_t Res;    
		Res =rx_buff_1;    //读取接收到的数据
		USART_RX_BUF_1[USART_RX_STA_1]=Res ;
		if(USART_RX_BUF_1[0]==0x80)
		{
				USART_RX_STA_1++;
				if( USART_RX_STA_1>2 )
				{
						if((USART_RX_BUF_1[1]==0x06) && (USART_RX_BUF_1[2]==0x83))
						{
								if(USART_RX_STA_1>=USART_REC_LEN) 
								{    
										USART_RX_STA_1=0;    
										if((USART_RX_BUF_1[3]!='E')&&(USART_RX_BUF_1[3]<0x34)&&(USART_RX_BUF_1[4]!='R')&&(USART_RX_BUF_1[5]!='R')
											&&(USART_RX_BUF_1[10]==(uint8_t)(~(0x80+0x06+0x83+USART_RX_BUF_1[3]+USART_RX_BUF_1[4]+USART_RX_BUF_1[5]+0x2E+USART_RX_BUF_1[7]+USART_RX_BUF_1[8]+USART_RX_BUF_1[9])+1)))  
										{                
												value[1] = (USART_RX_BUF_1[4]-0x30)*10000+(USART_RX_BUF_1[5]-0x30)*1000 + (USART_RX_BUF_1[7]-0x30)*100 + (USART_RX_BUF_1[8]-0x30)*10 + (USART_RX_BUF_1[9]-0x30);                 
												
										}    
								}
						}
						else USART_RX_STA_1=0;        
				}            
		} 
		else USART_RX_STA_1=0;            
}

