#ifndef __USART3_H__
#define __USART3_H__

#define REMOTE_DMA1_RX_BUF_LEN    100
#define Robot_State_Rx_Len        17          //机器人状态数据，应接受的数据量
#define Robot_Hurt_Rx_Len         10          //机器人伤害数据，应接受的数据量
#define Robot_PowerHeat_Rx_Len    29          //机器人功率热量数据，应接受的数据量
#define Robot_Shoot_Rx_Len        15          //机器人射击数据，应接受的数据量
void USART3_Init(void);
extern uint8_t REMOTE_DMA1_RX_BUF[2][REMOTE_DMA1_RX_BUF_LEN];
void UART3_PrintBlock(uint8_t* pdata, uint8_t len);//数组发送
void UART3_PrintBlock1(void);
#endif
