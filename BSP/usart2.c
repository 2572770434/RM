#include "main.h" 

#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式  
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
	USART3->DR = (u8) ch;      
	return ch;
}
#endif

__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN];//接收缓冲,最大USART_REC_LEN个字节.
u16 USART2_RX_STA=0;						 //接收状态标记	

#define miaosuan_data_dma_buf_len   9u
#define miaosuan_data_len      		 8u

#define send_buf_size 20
u8 sendbuf[send_buf_size];
float V_SPEED =0.0f;

static uint8_t miaosuan_data[2][miaosuan_data_dma_buf_len];
float camera_pitch =0;
float camera_yaw =0;
float camera_yaw_JSCOPE=0;
float camera_pitch_JSCOPE=0;
float CM_camera_yaw =0;
float CM_camera_pitch =0;
uint8_t stopSendFlag = 0;

float last_ChariotRecognition_pitch = 0.0f;
float error_add_average = 0.0f;
float last_camera_yaw = 0.0f;

float YawCurrentPositionSave   = 0.0f;   //保存当前Yaw轴位置
float PitchCurrentPositionSave = 0.0f;   //保存当前Pitch轴位置
float distance = 0.0f;

u8 recognize_flag = 0;
u8 enter_CNT = 0;

CRringBuffer_t CR_ringBuffer;

float CR_yaw_Angle[20];
u8 CR_yaw_Angle_Index = 0;
u8 CR_yaw_Angle_CNT   = 0;

int8_t loop_j;

void usart2_Init(u32 bound)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
	
	/* -------------- Configure GPIO & USART2 -------------------------------*/
	GPIO_InitTypeDef gpio;
	
	USART_InitTypeDef usart;
	DMA_InitTypeDef dma;
  NVIC_InitTypeDef nvic;	
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_Init(GPIOD, &gpio);
	
	USART_DeInit(USART2);
	USART_StructInit(&usart);
	usart.USART_BaudRate =bound;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_Mode = USART_Mode_Rx| USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &usart);
	
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	//USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
//	USART_ITConfig(USART2, USART_IT_TC, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	
	USART_Cmd(USART2, ENABLE);
	  
	/* -------------- Configure DMA1_ch4_stream5 PA3--------------------------------*/	
	DMA_DeInit(DMA1_Stream5);
	DMA_StructInit(&dma);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
	dma.DMA_Memory0BaseAddr = (uint32_t)&miaosuan_data[0][0];
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = sizeof(miaosuan_data)/2;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_Medium;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &dma);
		
	//配置Memory1,Memory0是第一个使用的Memory
	//下面的两句是开启双缓存模式
	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)&miaosuan_data[1][0], DMA_Memory_0);   //first used memory configuration
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	DMA_Cmd(DMA1_Stream5, ENABLE);

	nvic.NVIC_IRQChannel = USART2_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 3;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
	//串口2dma发送
//	DMA_DeInit(DMA1_Stream6);
//	while(DMA_GetCmdStatus(DMA1_Stream6)!= DISABLE);
//	dma.DMA_Channel = DMA_Channel_4;
//	dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
//	dma.DMA_Memory0BaseAddr = (uint32_t)sendbuf;
//	dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	dma.DMA_BufferSize = send_buf_size;
//	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	dma.DMA_Mode = DMA_Mode_Normal;
//	dma.DMA_Priority = DMA_Priority_Medium;
//	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA1_Stream6, &dma);
//	DMA_Cmd(DMA1_Stream6,ENABLE);
//	
//	DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
//	
//	nvic.NVIC_IRQChannel = DMA1_Stream6_IRQn;                          
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
//	nvic.NVIC_IRQChannelSubPriority = 4;		    //subpriority 
//	nvic.NVIC_IRQChannelCmd = ENABLE;			
//	NVIC_Init(&nvic);	
	
	
}

void dmasenddataproc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	DMA_Cmd(DMA_Streamx,DISABLE);
	while(DMA_GetCmdStatus(DMA_Streamx)!=DISABLE);
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);
	DMA_Cmd(DMA_Streamx,ENABLE);
}
void sendbytesinfoproc(u8*psendinfo,u16 nsendcount)
{
	u16 i=0;
	u8 *pbuf=NULL;
	pbuf=sendbuf;
	
	for(i=0;i<nsendcount;i++)
	{
		*pbuf++=psendinfo[i];
		
	}
	dmasenddataproc(DMA1_Stream6,nsendcount);
}
void DMA1_Stream6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)
	{
		DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
	}
}

void USART2_IRQHandler(void)
{
	static uint32_t usart2_this_time_rx_len = 0;
//	if(USART_GetFlagStatus(USART2,USART_FLAG_TC) !=RESET)
//	{
//		USART_ClearFlag(USART2,USART_FLAG_TC);
//	}
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		sendCustomDataFlag=1;
		//clear the idle pending flag 
		(void)USART2->SR;
		(void)USART2->DR;
		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			usart2_this_time_rx_len = miaosuan_data_dma_buf_len - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)miaosuan_data_dma_buf_len;     //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
      if(usart2_this_time_rx_len == miaosuan_data_len)
			{
				Camera_Mes_Process(miaosuan_data[0]);///顺藤摸瓜
			}
			Clear_Data(miaosuan_data[0]);
		}
		else //Target is Memory1
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			usart2_this_time_rx_len = miaosuan_data_dma_buf_len - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)miaosuan_data_dma_buf_len;      //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream5, ENABLE);
      if(usart2_this_time_rx_len == miaosuan_data_len)
			{
				Camera_Mes_Process(miaosuan_data[1]);
			}
			Clear_Data(miaosuan_data[1]);
		}
	}
	else
	{
		sendCustomDataFlag=0;
	}
}
/*
先横(水平)坐标,再纵(铅直)坐标,左上角为(0,0),max:(640,480)
*/
int16_t Identify_data[3];
int16_t CM_cameradata[3];
int16_t lastActionFlag = 0;
int16_t thisActionFlag = 0;
int CM_lostcount = 0;
float camera_yaw_add=0.0f;
//int16_t lastdistance= 0;
void Camera_Mes_Process(uint8_t *p)///////处理了一下摄像头传来的数据
{
	float camera_yaw_temp,camera_pitch_temp ;
	if(p[0] == 'R'&& p[1] == 'M'&&p[2] != 'A')
	{
		recognize_flag = 1;
		
		Identify_data[0] = (p[3]<<8) | p[2];
		Identify_data[1] = (p[5]<<8) | p[4]; 
		Identify_data[2] =  p[6];

		
		
		camera_yaw_temp = Identify_data[0];
		camera_yaw_add = camera_yaw_temp/100;
		
		camera_pitch_temp = Identify_data[1];
		camera_pitch = camera_pitch_temp/100;

    camera_yaw = camera_yaw_add;
		if(fabs(camera_yaw)>50)camera_yaw = 0;
		
		if(camera_yaw !=0)last_camera_yaw = camera_yaw;		
		camera_yaw = last_camera_yaw;

		
		//距离
		distance = Identify_data[2]*10;
	}
	

	
	if(p[0] == 'R'&& p[1] == 'M'&&p[2] == 'A' )
	{
    recognize_flag = 0;
		Identify_data[0] = 0;
		Identify_data[1] = 0; 
		Identify_data[2] =  0;

		camera_yaw = 0;
		camera_pitch = 0;
		distance =0;
		flag_Dafu_fromsight=0;
	}
	if(p[0] == 'B'&& p[1] == 'O')//接受大幅数据，
	{
		if(p[2] != 'A')
		{
		Identify_data[0] = (p[3]<<8) | p[2];
		Identify_data[1] = (p[5]<<8) | p[4];
		Identify_data[2] =  p[6];

		flag_Dafu_fromsight=1;

		
		camera_yaw_temp = Identify_data[0];
		camera_yaw_add = camera_yaw_temp/100;
		camera_pitch_JSCOPE = -camera_pitch_temp/100;//校正前
		camera_pitch_temp = Identify_data[1];
		camera_pitch = -(camera_pitch_temp/100)*0.7-0.16;//这个3是弹道强制补偿

    camera_yaw =  -camera_yaw_add;//-
		
		if(camera_yaw !=0)last_camera_yaw = camera_yaw;
		
		//距离
		distance = Identify_data[2]*10;
//		Jscope_test(camera_yaw,camera_yaw_temp,camera_yaw_add,camera_pitch_temp,camera_pitch);
		}
		else
		{
		recognize_flag = 0;
		Identify_data[0] = 0;
		Identify_data[1] = 0; 
		Identify_data[2] =  0;

		camera_yaw = camera_yaw;
		camera_pitch = camera_pitch;
		distance =distance;
		}
	}
}

void Clear_Data(uint8_t *p)
{
  int i=0;
	for(i=0;i<miaosuan_data_dma_buf_len;i++)
	{
		p[i]=0;
		i++;
	}
	
}
//BO  Y   P  distance  A
//01 23  45     6      7