#include "rabbit_action.h"
#include "rabbit_basic_action.h"
#include "main.h"
#include "com_chassis.h"
#include "vesc.h"

int get_circle_begin=0;//取环开始标志位
int clamp_situation_ok_flag=0;//环进入取环范围的标志位
int lauch_circle_begin=0;//进入发射状态时抬升开始工作标志位

int lift_target_total_angle=0;//抬升角度

//小兔的两个动作函数

//整个取环过程
int get_circle(void) //同时翻转，张开取环，抬升到位才返回1，否则返回0
{
	lauch_stop();//发射停止
	int res=0;
	res = overturn_down();
	res *= clamp_open();
	res *= lift_work_to_get_circle();
	return res;
}

//抬升3508和翻转3508同时复位
int overturn2zero(void)//同时复位，都到位后返回1
{
	int res=0;
	res = overturn_up();
	res *= lift_work_reset();
	return res;
}

void get_circle_all_action()
{
	while (get_circle_begin)
	{
		//各个过程是否完成的标志位，为1表示还未完成，为0为已完成
		static uint8_t get_circle_ok = 1, clamp_close_ok = 1, overturn2zero_ok = 1, clamp_loose_ok = 1;
		if (get_circle_ok && get_circle() == 0) break;
		com_finish_invert();//给底盘发消息，上层翻转完毕
		//翻转，取环，抬升都已到位
		get_circle_ok = 0;
		if (clamp_situation_ok_flag)
		{
			if (clamp_close_ok && clamp_close() == 0)break;
			clamp_close_ok = 0;
			com_close_finish();//给底盘发消息，上层夹环完毕
			if (overturn2zero_ok && overturn2zero() == 0)break;
			overturn2zero_ok = 0;
			HAL_Delay(400);
			if (clamp_loose_ok && clamp_loose() == 0)break;
			clamp_loose_ok = 0;

			//整个取环过程已完成
			clamp_situation_ok_flag = 0;
			get_circle_begin = 0;
			get_circle_ok = clamp_close_ok = overturn2zero_ok = clamp_loose_ok = 1;
		}
	}
}
uint32_t push_count_time1=0;
uint32_t push_count_time2=0;

void lauch_lift_work(int lauch_speed,int target_angle){//发射和抬升以及推环工作函数
	lauch_work(lauch_speed);//发射工作
	while(lauch_circle_begin){
		static uint8_t lift_work_ok = 1;
		static uint8_t first_push_count_flag=1;
		if(lift_work_ok && lift_work(target_angle)==0) break;//抬升工作
		lift_work_ok=0;
		push_count_time1=HAL_GetTick();
		if(first_push_count_flag){
			push_count_time2=push_count_time1;
			first_push_count_flag=0;
		}
		HAL_GPIO_WritePin(GPIOD, cylinder_contral_first_Pin, GPIO_PIN_SET);//气缸
		if(push_count_time1-push_count_time2>1500){
			HAL_GPIO_WritePin(GPIOD, cylinder_contral_first_Pin, GPIO_PIN_RESET);
			first_push_count_flag=1;
		}else{
			break;
		}
		lauch_circle_begin=0;
		lift_work_ok = 1;
	}
}

