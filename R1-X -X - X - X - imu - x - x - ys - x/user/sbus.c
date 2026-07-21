#include "sbus.h"

#include "usart.h"

#include "string.h"

#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00
#define SBUS_TX_BUF_SIZE 256

// Parameters related to receiving data / 接收数据相关参数
uint8_t sbus_start = 0;
uint8_t sbus_buf_index = 0;
uint8_t sbus_new_cmd = 0;
z
// data-caching mechanism / 数据缓存
uint8_t inBuffer[SBUS_RECV_MAX] = {0};
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;

uint8_t j;
uint8_t sbus_data[SBUS_RECV_MAX] = {0};
static uint8_t sbus_dma_buf[SBUS_RECV_MAX] = {0};
static uint8_t sbus_tx_buf[SBUS_TX_BUF_SIZE] = {0};
static volatile uint16_t sbus_tx_head = 0;
static volatile uint16_t sbus_tx_tail = 0;
static volatile uint16_t sbus_tx_dma_len = 0;
static volatile uint8_t sbus_tx_busy = 0;
int16_t g_sbus_channels[18] = {
    992, 992, 992, 992, 192, 192, 192, 192, 192,
    192, 192, 192, 192, 192,192, 192, 192, 192
};

// 解析 SBUS 数据
static int SBUS_Parse_Data(void)
{
    g_sbus_channels[0]  = ((sbus_data[1] | sbus_data[2] << 8) & 0x07FF);
    g_sbus_channels[1]  = ((sbus_data[2] >> 3 | sbus_data[3] << 5) & 0x07FF);
    g_sbus_channels[2]  = ((sbus_data[3] >> 6 | sbus_data[4] << 2 | sbus_data[5] << 10) & 0x07FF);
    g_sbus_channels[3]  = ((sbus_data[5] >> 1 | sbus_data[6] << 7) & 0x07FF);
    g_sbus_channels[4]  = ((sbus_data[6] >> 4 | sbus_data[7] << 4) & 0x07FF);
    g_sbus_channels[5]  = ((sbus_data[7] >> 7 | sbus_data[8] << 1 | sbus_data[9] << 9) & 0x07FF);
    g_sbus_channels[6]  = ((sbus_data[9] >> 2 | sbus_data[10] << 6) & 0x07FF);
    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3) & 0x07FF);
    #ifdef ALL_CHANNELS
    g_sbus_channels[8]  = ((sbus_data[12] | sbus_data[13] << 8) & 0x07FF);
    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5) & 0x07FF);
    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10) & 0x07FF);
    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7) & 0x07FF);
    g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4) & 0x07FF);
    g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9) & 0x07FF);
    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6) & 0x07FF);
    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3) & 0x07FF);
    #endif

    // 安全检测：判断是否失联或数据异常
    // Security detection: check for lost connection or data errors
    failsafe_status = SBUS_SIGNAL_OK;
    if (sbus_data[23] & (1 << 2))
    {
        failsafe_status = SBUS_SIGNAL_LOST;
        // Lost contact / 失联
    }
    else if (sbus_data[23] & (1 << 3))
    {
        failsafe_status = SBUS_SIGNAL_FAILSAFE;
        // Data loss / 数据丢失
    }
    return failsafe_status;
}

void SBUS_Start_DMA(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_dma_buf, SBUS_RECV_MAX) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

static void SBUS_TxKick(void)
{
    if (sbus_tx_busy || sbus_tx_head == sbus_tx_tail)
    {
        return;
    }

    sbus_tx_busy = 1;
    if (sbus_tx_head > sbus_tx_tail)
    {
        sbus_tx_dma_len = (uint16_t)(sbus_tx_head - sbus_tx_tail);
    }
    else
    {
        sbus_tx_dma_len = (uint16_t)(SBUS_TX_BUF_SIZE - sbus_tx_tail);
    }

    if (HAL_UART_Transmit_DMA(&huart3, &sbus_tx_buf[sbus_tx_tail], sbus_tx_dma_len) != HAL_OK)
    {
        sbus_tx_busy = 0;
    }
}

uint8_t SBUS_Send_DMA(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len >= SBUS_TX_BUF_SIZE)
    {
        return 0;
    }

    __disable_irq();
    uint16_t head = sbus_tx_head;
    uint16_t tail = sbus_tx_tail;
    uint16_t used = (head >= tail) ? (head - tail) : (SBUS_TX_BUF_SIZE - tail + head);
    uint16_t free_space = (uint16_t)(SBUS_TX_BUF_SIZE - 1 - used);
    if (len > free_space)
    {
        __enable_irq();
        return 0;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        sbus_tx_buf[head] = data[i];
        head++;
        if (head >= SBUS_TX_BUF_SIZE)
        {
            head = 0;
        }
    }
    sbus_tx_head = head;
    __enable_irq();

    SBUS_TxKick();
    return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
        for (uint16_t i = 0; i < Size; i++)
        {
            SBUS_Reveive(sbus_dma_buf[i]);
        }
        SBUS_Start_DMA();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        sbus_tx_tail = (uint16_t)(sbus_tx_tail + sbus_tx_dma_len);
        if (sbus_tx_tail >= SBUS_TX_BUF_SIZE)
        {
            sbus_tx_tail = (uint16_t)(sbus_tx_tail - SBUS_TX_BUF_SIZE);
        }
        sbus_tx_busy = 0;
        SBUS_TxKick();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        (void)HAL_UART_AbortReceive(&huart3);
        SBUS_Start_DMA();
    }
}
// Receives SBUS cached data / 接收 SBUS 缓存数据
void SBUS_Reveive(uint8_t data)
{
    // If the protocol start flag is met, start receiving data / 检测到起始标志后开始接收
    if (sbus_start == 0 && data == SBUS_START)
    {
        sbus_start = 1;
        sbus_new_cmd = 0;
        sbus_buf_index = 0;
        inBuffer[sbus_buf_index] = data;
        inBuffer[SBUS_RECV_MAX - 1] = 0xff;
    }
    else if (sbus_start)
    {
        sbus_buf_index++;
        inBuffer[sbus_buf_index] = data;
    }

    // Finish receiving a frame of data / 接收完成一帧数据
    if (sbus_start & (sbus_buf_index >= (SBUS_RECV_MAX - 1)))
    {
        sbus_start = 0;
        if (inBuffer[SBUS_RECV_MAX - 1] == SBUS_END)
        {
            memcpy(sbus_data, inBuffer, SBUS_RECV_MAX);
            sbus_new_cmd = 1;
        }
    }
}

// SBUS receive & process handler / SBUS 接收与处理
void SBUS_Handle(void)
{
    if (sbus_new_cmd)
    {
        int res = SBUS_Parse_Data();
			
        sbus_new_cmd = 0;
        if (res) return;
//        #if SBUS_ALL_CHANNELS
//        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
//			   g_sbus_channels[3], g_sbus_channels[4], g_sbus_channels[5],
//			   g_sbus_channels[6], g_sbus_channels[7], g_sbus_channels[8],
//			   g_sbus_channels[9], g_sbus_channels[10], g_sbus_channels[11],
//			   g_sbus_channels[12], g_sbus_channels[13], g_sbus_channels[14],
//			   g_sbus_channels[15]);
//        #else
//        printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",
//               g_sbus_channels[0], g_sbus_channels[1], g_sbus_channels[2],
//			   g_sbus_channels[3], g_sbus_channels[4],g_sbus_channels[5],
//			   g_sbus_channels[6], g_sbus_channels[7]);
//        #endif
    }
}

void sbus_loop(void)
{
	SBUS_Handle();
//	if(g_sbus_channels[1]>995&&g_sbus_channels[1]<1010)
//		g_sbus_channels[1]=1000;
//	if(g_sbus_channels[1]>1750)
//		g_sbus_channels[1]=1800;
//	if(g_sbus_channels[1]<200)
//		g_sbus_channels[1]=200;
	//HAL_Delay(1);
	
}

// 返回当前 SBUS 状态：SBUS_SIGNAL_OK / SBUS_SIGNAL_LOST / SBUS_SIGNAL_FAILSAFE
uint8_t SBUS_GetStatus(void)
{
    return failsafe_status;
}

// 返回底层通道指针（可读）
int16_t* SBUS_GetChannels(void)
{
    return g_sbus_channels;
}
/**** 红外发射（YS-IRTM，USART2）   ****/
void YS_IRTM_Send(uint8_t key)
{
    uint8_t frame[5] = {YS_FA, YS_F1, YS_one, YS_two, key};
    HAL_UART_Transmit(&huart2, frame, 5, 100);
}