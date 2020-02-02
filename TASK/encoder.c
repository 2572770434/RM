#include "main.h"

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg)
{
		v->ecd_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
		v->raw_value= v->ecd_bias;
		v->ecd_value = v->ecd_bias;
}

void Motor_2310_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*(360/36.0)/8192 + v->round_cnt * 360/36.0;//减速比36:1
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);			
}

void Motor_6623_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;
  int16_t torque_current6623 = 0;	
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;//+ v->round_cnt * 360;
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
	torque_current6623= (msg->Data[4]<<8)|msg->Data[5];
	v->real_torque_current = torque_current6623;
}

void Motor_6020_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;
  int16_t torque_current6623 = 0;
  int16_t motor_speed = 0;	
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;//+ v->round_cnt * 360;
	v->ecd_xtl_angle = (double)(v->raw_value - v->ecd_bias)*360/8192 ;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	motor_speed = (msg->Data[2]<<8)|msg->Data[3];
	v->filter_rate = motor_speed *360/60;	
	torque_current6623= (msg->Data[4]<<8)|msg->Data[5];
	v->real_torque_current = torque_current6623;
}

void Motor_3510_EncoderProcess(volatile Encoder *v, CanRxMsg * msg)
{
	int i=0;
	int32_t temp_sum = 0;
  int16_t temp_filter=0;	
	int16_t torque_current = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//保存diff
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (double)(v->raw_value - v->ecd_bias)*(360/19.0)/8192 + v->round_cnt * 360/19.0;//减速比36:1
	
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	//v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);	
	temp_filter = ((msg->Data[2]<<8)|msg->Data[3]);
	v->filter_rate = temp_filter/19;
	torque_current=((msg->Data[4]<<8)|msg->Data[5]);
	v->real_torque_current = torque_current;
}
