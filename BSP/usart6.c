#include "main.h"

/*-----USART6_TX-----PC6-----*/
/*-----USART6_RX-----PC7----*/

static uint8_t IMU_DMA_RX_BUF[2][IMU_DMA_RX_BUF_LEN];
uint8_t angle[12]={0};
int sendflag =0;

void USART6_Configuration(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* -------------- Configure GPIO & USART6 -------------------------------*/
	{
		GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
		
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);
	
    gpio.GPIO_Pin = GPIO_Pin_9 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOG,&gpio);
	
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6,&usart);

		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
		USART_Cmd(USART6,ENABLE);
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	}
		/* -------------- Configure DMA2_Stream2 --------------------------------*/
	{
		DMA_InitTypeDef dma;
		
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_5;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&IMU_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(IMU_DMA_RX_BUF)/2;
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
    DMA_Init(DMA2_Stream2, &dma);
    
    //配置Memory1,Memory0是第一个使用的Memory
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&IMU_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    
    DMA_Cmd(DMA2_Stream2, ENABLE);
	}
	/* -------------- Configure NVIC ----------------------------------------*/
	{
		NVIC_InitTypeDef  nvic;
		
		nvic.NVIC_IRQChannel = USART6_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);
	}
}

//void USART6_SendChar(char b)
//{
//    while( USART_GetFlagStatus(USART6,USART_FLAG_TC) == RESET);
//	USART_SendData(USART6,b);
//}

void USART6_IRQHandler(void)                	//串口6中断服务程序
{
	static uint32_t usart6_this_time_rx_len = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		//clear the idle pending flag 
		(void)USART6->SR;
		(void)USART6->DR;

		//Target is Memory0
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			usart6_this_time_rx_len = IMU_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)IMU_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(usart6_this_time_rx_len == IMU_FRAME_LENGTH)
			{
				IMUDataProcess(IMU_DMA_RX_BUF[0]);
			}
		}
		else //Target is Memory1
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			usart6_this_time_rx_len = IMU_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)IMU_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
      if(usart6_this_time_rx_len == IMU_FRAME_LENGTH)
			{
				IMUDataProcess(IMU_DMA_RX_BUF[1]);
			}
		}
	}       
}

float Angles;
int16_t Gyro[3];
float Eular[3];     //欧拉角
//九轴有加速度计\磁力计\陀螺仪总共九个数据,而且芯片自带滤波处理,直接输出欧拉角.
int16_t p;
int16_t y;
void IMUDataProcess(uint8_t *pData)    //IMU其实是解算姿态的
{
	
		static  int   count     = 0;
		static  float LastAngle = 0;
		static  float NowAngle = 0;
	
		if(pData == NULL)
		{
			return;
		}
		Gyro[0] = ((float)(int16_t)(pData[16] + (pData[17]<<8)));//roll
		Gyro[1] = ((float)(int16_t)(pData[18] + (pData[19]<<8)));//pitch
		Gyro[2] = ((float)(int16_t)(pData[20] + (pData[21]<<8)));//yaw

		Eular[0] = ((float)(int16_t)(pData[30] + (pData[31]<<8)))/100;//
		Eular[1] = -((float)(int16_t)(pData[32] + (pData[33]<<8)))/100;//pitch	
		
		Angles = ((float)(int16_t)(pData[34] + (pData[35]<<8)))/10;//yaw
		LastAngle=NowAngle;
		NowAngle=Angles;
		if((NowAngle-LastAngle)>300)
			count--;
		if((NowAngle-LastAngle)<-300)
			count++;
		
		Eular[2]  =(NowAngle+count*360);//
		
//		p=((GMPitchEncoder.ecd_angle-0.3)*0.77153 +90)*10;//-Eular[0]
//		y= (-Angles+180)*10;
		p=(-GMPitchEncoder.ecd_angle+90)*10;
		y=(-GMYawEncoder.ecd_angle+180) *10;
angle[0]='M';
//    angle[1]='N';
//		angle[2]='L';
		
	  angle[1]= (uint8_t)(p/1000+48);			//+48的原因是ASCII码
    angle[2]= 	(uint8_t)(p%1000/100+48);
		angle[3]= 	(uint8_t)(((p%1000)%100)/10+48);
		angle[4]= 	(uint8_t)(((p%1000)%100)%10+48);
	  angle[5]= (uint8_t)(y/1000+48);
	  angle[6]= (uint8_t)(y%1000/100+48);
		angle[7]= 	(uint8_t)((y%1000)%100/10+48);
		angle[8]= 	(uint8_t)(((y%1000)%100)%10+48);
//	  if(robotState.robot_id == 0x03) angle[11] = 'B';
//		else 
		angle[9] = 'R';//(recognizecolor == BLUE) 
		
		
//		angle[8]= '\n';
}

int h=0;
void SenddatatoNUC(void)
{
	system_micrsecond.time_now = Get_Time_Micros();
	system_micrsecond.time_error = system_micrsecond.time_now - system_micrsecond.time_last;
	if(system_micrsecond.time_error > 1000)//1ms的间隔
	{
	 if( predictflag==1&&dafu_temp != 1)
	 {
/*		 if(quickShootFlag == 2)
//		 {
//			 angle[2]='H';
//		 }
//		 else 
//		 {
//			 angle[2]='L';
//		 }
				angle[1]='Y';*/		
		 for(h=0;h<12;h++)
			{
				
				USART_SendData(USART2,angle[h]);
			//	delay_us(20);
				while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0);
			}
	 }
	 else if(dafu_temp != 1)
	 {
/*		 if(quickShootFlag == 2)
		 {
			 angle[2]='H';
		 }
		 else 
		 {
			 angle[2]='L';
		 }*/
		for(h=0;h<12;h++)
		{
			
			USART_SendData(USART2,angle[h]);
			//delay_us(20);
  			while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0);
		}
	}
	 else if( dafu_temp == 1)//发打符信号给视觉
	 {//for(h=0;h<10;h++)
		/*angle[1]='O';
		angle[2]='B';*/
		for(h=0;h<12;h++)
		{
			USART_SendData(USART2,angle[h]);
			//delay_us(20);
			while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==0);
		}
//		{
//			USART_SendData(USART2,'B');
//			//while(USART_GetITStatus(USART2, USART_FLAG_TXE) != RESET);
//			delay_us(100);
//			USART_SendData(USART2,'O');
//		  delay_us(100);
//		}
	 }	
 }
}
