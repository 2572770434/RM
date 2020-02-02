#include "main.h"
WorkState_e lastWorkState = STOP_STATE;
WorkState_e workState = STOP_STATE;
uint8_t camera_temp = 0;
uint8_t dafu_temp = 0;
uint8_t cat_temp = 0;    
//int identify_flag =0 ;      //ÓÃÓÚÉèÖÃÊÇ·ñÎªÕ½³µÊ¶±ðÄ£Ê½
/**
º¯Êý£ºWorkStateFSM()
¹¦ÄÜ£º¿ØÖÆ¹¤×÷Ä£Ê½
**/
/////////////////////ÒÔÏÂÊÇÁ¢flagµÄÇøÓò
int flag_prepare_ready=0;
int flag_Dafu_fromsight=1;//0;//´ÓÊÓ¾õ·½Ãæ½ÓÊÕµ½ÁË´ó·ùÊý¾Ý
////////////////////
void WorkStateFSM(void)
{
	lastWorkState = workState;//¸üÐÂµ±Ç°¹¤×÷×´Ì¬
	switch(workState)
	{
		case STOP_STATE:                            
		{
			if(GetInputMode() != STOP)//Ò£¿ØÆ÷Ç¿ÖÆÍ£Ö¹
			{
				workState = PREPARE_STATE;   //È¡Ïû¼±Í£ÒªÏÈ¹ý¶¶É
			}
		}break;
		
		case PREPARE_STATE://×¼±¸×´Ì¬
		{
			if(GetInputMode() == STOP)//Ò£¿ØÆ÷Ç¿ÖÆÍ£Ö¹
			{
				workState = STOP_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)//×¼±¸×´Ì¬´ïµ½Ö¸¶¨Ê±¼äºó£¬×ªµ½²Ù×÷ÊÖ¿ØÖÆ×´Ì¬
			{
				workState = NORMAL_STATE;
			}
		}break;
		case NORMAL_STATE:     //²Ù×÷ÊÖ¿ØÖÆ×´Ì¬£¨ÓÐÊäÈë£©
		{
			if(GetInputMode() == STOP)//Ò£¿ØÆ÷Ç¿ÖÆÍ£Ö¹
			{
				workState = STOP_STATE;
			}
//			else if(camera_temp == 1)//ÞôÏÂc¼ü£¬½øÈë´ò´ó·ûÄ£Ê½
//			{
////       S1inputmode = ZIMIAO;
//			}
			else if(dafu_temp == 1&&flag_Dafu_fromsight==1)//ÞôÏÂf¼ü£¬×ªµ½ÎÞÊäÈë×´Ì¬    (´ò´ó¸£) 
			{
				workState = CAMERA_STATE;//STANDBY_STATE;      
			}	
			else if(cat_temp == 1)//Ò£¿ØÆ÷¡¢¼üÅÌÎÞÊäÈë£¬×ªµ½ÎÞÊäÈë×´Ì¬     
			{
				workState = CATWALK_STATE;///CAMERA_STATE;//STANDBY_STATE;      
			}	
		}break;
		
		case CAMERA_STATE:      //´ó·û×´Ì¬
		{
			if(GetInputMode() == STOP)//Ò£¿ØÆ÷Ç¿ÖÆÍ£Ö¹
			{
				workState = STOP_STATE;
			}	
			else if(dafu_temp == 0)//ËÉ¿ªC¼ü£¬½øÈëµ×ÅÌ²»¶¯×´Ì¬
			{
				workState = NORMAL_STATE;      
			}
		}break;
	
		case ADJUST_STATE:      //Ãé×¼×´Ì¬
		{
//			if(GetInputMode() == STOP)
//			{
//				workState = STOP_STATE;
//			}
//			else if(adjust_temp == 0)
//			{
//				workState = NORMAL_STATE;      
//			}
		}break;
		
		case CATWALK_STATE:      
		{
			if(GetInputMode() == STOP)
			{
				workState = STOP_STATE;
			}
			else if(cat_temp == 0)
			{
				workState = NORMAL_STATE;      
			}
		}break;
		default:
		{
		}
	}	
}

/**
º¯Êý£ºWorkStateSwitchProcess()
¹¦ÄÜ£ºÈç¹û´ÓÆäËûÄ£Ê½ÇÐ»»µ½prapareÄ£Ê½£¬Òª½«Ò»ÏµÁÐ²ÎÊý³õÊ¼»¯
**/
void WorkStateSwitchProcess(void)
{
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		ControtLoopTaskInit();
		RemoteTaskInit();
		ChassisMotorInit();
		//AllDataInit();
	}
}
/**
º¯Êý£ºSetWorkState(WorkState_e state)
¹¦ÄÜ£º·µ»Øµ±Ç°¹¤×÷×´Ì¬

**/
void SetWorkState(WorkState_e state)
{
	if(workState != PREPARE_STATE)
	{
    workState = state;
	}
}

/**
º¯Êý£ºWorkState_e GetWorkState(void)
¹¦ÄÜ£ºÉèÖÃ¹¤×÷×´Ì¬

**/
WorkState_e GetWorkState(void)
{
	return workState;
}

WorkState_e GetlastWorkState(void)
{
	return lastWorkState;
}

void AllDataInit(void)
{
	Key_State cameraKey = KEY_INIT;//zimiaoû¼ü
	Key_State dafuKey = KEY_INIT;//´ó·û¼ü
	Key_State servoKey = KEY_INIT;//¶æ»ú¼ü
	Key_State frictionKey = KEY_INIT;//Ä¦²ÁÂÖËÙ¶ÈÇÐ»»¼ü
	uint8_t mouseLeftPress = 0;
	uint8_t continuous_shoot_Flag = 0;
	uint8_t quickShootFlag = 0;
	uint8_t servoPositionFlag = 0;
	uint8_t sendCustomDataFlag = 0;
	uint8_t camera_temp = 0;
	uint8_t dafu_temp = 0;
	uint8_t cat_temp = 0;
	Key_State xtl = KEY_INIT;//XTL
}
