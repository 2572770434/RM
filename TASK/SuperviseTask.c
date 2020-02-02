#include "main.h"

uint16_t wwdg_flag = 0xffff;
 
/**********************************
* FUNCTION NAME：WWDG_Init
* FUNCTION ：看门狗功能初始化
* MODIFY DATE ：2017
* INPUT ：无
* RETURN ：无
***********************************/
void WWDG_Init(void)
{
	/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE); //使能窗口看门狗时钟
	
	/* -------------- Enable The WWDG ---------------------------------------*/
	
		WWDG_SetPrescaler(WWDG_Prescaler_8); //设置分频值
		WWDG_SetWindowValue(0x42); //设置窗口值  0100 0010 
		//0x40是下窗口的值,上窗口的值自己定
		WWDG_Enable(0x43);  //开启看门狗			   0100 0011
		WWDG_ClearFlag();//清除提前唤醒中断标志位
	
}

/**********************************
* FUNCTION NAME：ShootSupervise
* FUNCTION ：波弹电机监视程序，防止卡弹
* MODIFY DATE ：2018.3.14
* INPUT ：无
* RETURN ：无
***********************************/
void ShootSupervise(void)
{
	float temp;
		temp = (CM7Encoder.ecd_angle/51.4285714 +0.1+ count_temp);
		if(temp>-2)  
	{
	}
		else
	{
		count_temp = (int16_t)(CM7Encoder.ecd_angle/51.4285714 -2.1);
	}
}

/**********************************
* FUNCTION NAME：SuperiviseTask
* FUNCTION ：波弹电机监视程序，
* MODIFY DATE ：2017
* INPUT ：无
* RETURN ：0 1（目前不清楚为什么要这么写）
***********************************/
int SuperiviseTask(void)
{
	/* -------------- 检查遥控器和mpu6050 -----------------------------------*/
	{
		mpu6050_micrsecond.time_now   = Get_Time_Micros();
		mpu6050_micrsecond.time_error = mpu6050_micrsecond.time_now - mpu6050_micrsecond.time_last;
		remote_micrsecond.time_now    = Get_Time_Micros();
		remote_micrsecond.time_error  = remote_micrsecond.time_now - remote_micrsecond.time_last;
		wwdg_flag &= ~0x0180;
		if(remote_micrsecond.time_error > 14500)//遥控器手册上写的数据传送周期为7ms,但是实测为14ms
		{
			wwdg_flag |= 0x0080;
		}
		if(remote_micrsecond.time_error > 6500)//mpu6050数据传输速度,实测为4000~6000ms(受别的中断影响)
		{
			wwdg_flag |= 0x0100;
		}
	}
	/* -------------- 任务正常,则喂狗 ---------------------------------------*/
	{
		if((wwdg_flag & 0x01ff) ==0x0000)      
		{
		 WWDG_SetCounter(0x43);             //喂狗
			wwdg_flag = 0;
			return 1;
		}
		else
		{
			return 0;
		}
	}
}
