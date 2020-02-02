#include "main.h"
#include "arm_math.h" 
void BSP_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	TIM2_Configuration();
	TIM6_Configuration();
	CAN1_Configuration();
	CAN2_Configuration();
	USART1_Configuration();
	USART3_Init();//115200
	usart2_Init(115200);
	USART6_Configuration();
	PWM_Configuration();
	USART7_Configuration();
	CreateKalman(&KALMAN1);
	InitKalman(&KALMAN1);
	laser_configuration();
//	kalman_filter_init(&KALMAN1, &KALMAN2);
}

void laser_configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;      //??????g?
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //???????
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //????
    GPIO_Init(GPIOG, &GPIO_InitStructure);             //??'??

    laser_on();
}
void laser_on(void)
{
    GPIO_SetBits(GPIOG, GPIO_Pin_13);
}
void laser_off(void)
{
    GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}
