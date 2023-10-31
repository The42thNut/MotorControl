#include "rabbit_basic_action.h"


//发射函数
void lauch_work(int speed){
	set_vesc_speed(0x001,speed);
	set_vesc_speed(0x002,speed);
}

void lauch_stop(void){
	set_vesc_speed(0x001,0);
	set_vesc_speed(0x002,0);
}

//翻转函数
int overturn_up(void){//翻转到初始位置
	set_loc[overturn_3508]=-motor_angle_error;
	if(motor_chassis[overturn_3508].total_angle>-motor_angle_error- motor_angle_error){
		return 1;
	}else{
		return 0;
	}
}

int overturn_down(void){//翻转下去取环
	set_loc[overturn_3508]=overturn_angle;
	if(motor_chassis[overturn_3508].total_angle< overturn_angle + motor_angle_error)
		return 1;
	else
		return 0;
}
//夹环函数
int clamp_open(void){//取环打开到最大
	set_loc[clamp_2006]=0;
	if(motor_chassis[clamp_2006].total_angle< motor_angle_error)
		return 1;
	else
		return 0;
}

int clamp_loose(void){//放环
	set_loc[clamp_2006] = clamp_loose_distance;
	if (motor_chassis[clamp_2006].total_angle < clamp_loose_distance + motor_angle_error)
		return 1;
	else
		return 0;
}

int clamp_close(void){//取环夹紧
	set_loc[clamp_2006]=clamp_close_distance;
	if (motor_chassis[clamp_2006].total_angle > clamp_close_distance - motor_angle_error)
		return 1;
	else
		return 0;
}
//抬升函数
int lift_work_to_get_circle(void){//抬升3508配合取环转动函数
	set_loc[lift_3508]=lift_get_circle_angle;
	if (motor_chassis[lift_3508].total_angle < lift_get_circle_angle + motor_angle_error)
		return 1;
	else
		return 0;
}

int lift_work_reset(void){//抬升3508回到初始位置
	set_loc[lift_3508]=-100;
	if(motor_chassis[lift_3508].total_angle>-400)
		return 1;
	else
		return 0;
}

int lift_work(int target_angle){//抬升角度，未作返回值处理
	set_loc[lift_3508]=target_angle;
	if(motor_chassis[lift_3508].total_angle<target_angle){
		if(motor_chassis[lift_3508].total_angle > target_angle*0.95){
			return 1;
		}else{
			return 0;
		}
	}else{
		if(motor_chassis[lift_3508].total_angle < target_angle*1.05){
			return 1;
		}else{
			return 0;
		}
	}
}

//推环函数
int judge_lift_finish(int target_angle){//判断抬升是否到位
	if(motor_chassis[lift_3508].total_angle<target_angle){
		if(motor_chassis[lift_3508].total_angle > target_angle*0.95){
			return 1;
		}else{
			return 0;
		}
	}else{
		if(motor_chassis[lift_3508].total_angle < target_angle*1.05){
			return 1;
		}else{
			return 0;
		}
	}
}
