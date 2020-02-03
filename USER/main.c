#include "main.h"
int cou=0;

int i=0;
int main()
{
  delay_ms(2000);
	engineerpower_Init();
	BSP_Init();

	ControtLoopTaskInit();

	RemoteTaskInit();
	delay_ms(1000);

	TIM6_Start();
	//WWDG_Init();

	delay_ms(100);

	while(1)
	{
	
	  delay_ms(1);
		

		cou++;

	}
}
