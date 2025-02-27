#include "main.h"

int recieve=0;
unsigned char buff[5];
int16_t jiesuan_data[3];
 int shu=0;
void USART7_Configuration(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE); 
	
	/* -------------- Configure GPIO & USART2 -------------------------------*/
	GPIO_InitTypeDef gpio;
	
	USART_InitTypeDef usart;
	DMA_InitTypeDef dma;
  NVIC_InitTypeDef nvic;	
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);
	
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &gpio);
	
	USART_DeInit(UART7);
  USART_StructInit(&usart);////所有通讯设置都在这里集成了

	USART_Init(UART7, &usart);
	
	
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
	USART_ITConfig(UART7, USART_IT_TC, ENABLE);
	USART_Cmd(UART7, ENABLE);
	  
	

	nvic.NVIC_IRQChannel = UART7_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 1;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	
	
}

float exchange(int bai,int shi,int ge,int fu1,int fu2,int fu3)
{float result=0;
	result=100*(bai)+10*(shi)+ge+0.1*(fu1)+0.01*(fu2)+0.001*(fu3);
	return result;
}
	

void UART7_IRQHandler(void)
{
	static uint32_t usart7_this_time_rx_len = 0;
	
//	if(j>4)j=0;
	if(USART_GetFlagStatus(UART7,USART_FLAG_TC) != RESET)
	{

	USART_ClearFlag(UART7,USART_FLAG_TC);
	}

	
		if(USART_GetFlagStatus(UART7,USART_FLAG_RXNE) != RESET)//USART_GetITStatus
	{
		static int  j=0;
		USART_ClearFlag(UART7,USART_IT_RXNE);
		recieve=USART_ReceiveData( UART7);
	
		
		buff[shu]=recieve;
		shu++;
		if(shu>3)
			shu=0;
		
		if(buff[0]=='R'&&buff[1]=='M')
	{
		jiesuan_data[0] = (buff[3]<<8) | buff[2];
		
	}
		//USART_SendData( UART7,recieve);////////////收完数立刻发出去

		/////////////测压模块的数据筛选，晒除掉//////
		//		if((recieve!=0x0D)&&(recieve!=0x0A)&&(recieve!=0x4F)&&(recieve!=0x4B)&&(recieve!=0x56)&&(recieve!=0x2B)&&(recieve!=0x3D))
//		{RE[j]=recieve;
//			j++;
//		}
//		if(recieve==0x3D)
//		{judge=1;return;
//		}
//		if(judge==1)
//		{
//			if(recieve==0x2B)return;
//			jieguo[i]=recieve;
//			i++;
//			if(i>1)
//			{		if(jieguo[1]==0x2E)
//					{RESULT=jieguo[0]-0x30;
//					}
//					else
//					{RESULT=10*(jieguo[0]-0x30)+(jieguo[1]-0x30);
//					}
//		
//					i=0;
//					judge=0;
//			}
//		}
		
//			if((recieve!=0x0D)&&(recieve!=0x0A)&&(recieve!=0x4F)&&(recieve!=0x4B)&&(recieve!=0x56)&&(recieve!=0x2B)&&(recieve!=0x3D))
//		{RE[i]=recieve;
//			i++;
//		}
		
//		if(i>5)
//		{cha[0]=RE[0]-0x30;
//			cha[1]=RE[1]-0x30;
//		cha[2]=RE[3]-0x30;
//			cha[3]=RE[4]-0x30;
//			cha[4]=RE[5]-0x30;
//			RESULT=exchange(0,cha[0],cha[1],cha[2],cha[3],cha[4]);
//			i=0;
//		}
		
	
		}
	

	
	
	

}

void uart_test(void)
{
uint16_t temp;
signed    char tempp;//int_8
	USART_SendData( USART2, 0xA5);
//uart_putchar(0xA5);
////check

	USART_SendData( USART2, 0);
	
//uart_putchar((uint8_t)(temp>>8));
//uart_putchar((uint8_t)(temp));
	
tempp = CM1SpeedPID.ref*0.05;
	USART_SendData( USART2, 0x03);
//uart_putchar((uint8)(temp>>8));
//uart_putchar((uint8)(temp));
//angle_v
tempp = -0.05*CM1SpeedPID.fdb; 

	USART_SendData( USART2, 0x06);
//uart_putchar((unsigned char)(temp>>8));
//uart_putchar((unsigned char)(temp));
//angle_z
//tempp = (int16)acc.angle_z;
//temp = tempp;
//uart_putchar((uint8)(temp>>8));
//uart_putchar((uint8)temp);
USART_SendData( USART2, 0xA1);
//uart_putchar(0xAA);???????????
}

