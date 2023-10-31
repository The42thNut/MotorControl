#include "pid.h"

/**************内部变量与函数begin**************/
/*增量式pid初始化*/
static void Pid_increment_struct_init(pid_incremental_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd, ElemType out_limit_up, ElemType out_limit_down);
/**************内部变量与函数end**************/


/**************外部接口begin**************/
motor_pid_parameter motor_pid_g[8];/*电机pid参数*/
void Pid_parameter_init(void);/*各套pid参数初始化参数,必须先调用*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target);/*增量式pid计算*/
/**************外部接口end**************/

/*
1.函数功能：初始化一套pid中的各参数
2.入参：结构体指针，kp，ki，kd，输出上限，输出下限
3.返回值：none
4.用法及调用要求：
5.其它：
*/
static void Pid_increment_struct_init(pid_incremental_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd, ElemType out_limit_up, ElemType out_limit_down) {
	pid_struct->Kp = Kp;
	pid_struct->Ki = Ki;
	pid_struct->Kd = Kd;

	pid_struct->last_out = 0;

	pid_struct->err = 0;
	pid_struct->err_last = 0;
	pid_struct->err_last_last = 0;

	pid_struct->out_limit_up = out_limit_up;
	pid_struct->out_limit_down = out_limit_down;
	
	pid_struct->now_out = 0;
}
/*
1.函数功能：各套pid参数初始化参数
2.入参：none
3.返回值：none
4.用法及调用要求：在使用电机之前必须先调用此函数进行pid参数初始化
5.其它：
*/
void Pid_parameter_init(void){
	Pid_increment_struct_init(&motor_pid_g[0].loc,  0.15, 	0.001,	0.035,   50,    -50);
	Pid_increment_struct_init(&motor_pid_g[0].spd,  24.0, 	1.2,	  0.03,  12000,  -12000);
	
	Pid_increment_struct_init(&motor_pid_g[1].loc,  0.15, 	0.001,	0.035,   50,    -50);
	Pid_increment_struct_init(&motor_pid_g[1].spd,  24.0, 	1.2,	  0.03,  12000,  -12000);
	
	Pid_increment_struct_init(&(motor_pid_g[2].loc),  0.15, 	0.001,	0.035,   50,    -50);
	Pid_increment_struct_init(&(motor_pid_g[2].spd),  24.0, 	1.2,	  0.03,  9500,  -9500);
	
//	Pid_increment_struct_init(&motor_pid_g[3].loc,  0.364706f,	0.007333f,	0.004745f,   500,    -500);
//	Pid_increment_struct_init(&motor_pid_g[3].spd,  16.7f,	0.98f,	0.024314f,  12000,  -12000);
	
	Pid_increment_struct_init(&motor_pid_g[3].loc,  0.15, 	0.001,	0.035,   50,    -50);
	Pid_increment_struct_init(&motor_pid_g[3].spd,  24.0, 	1.2,	  0.03,  9500,  -9500);
	
	Pid_increment_struct_init(&motor_pid_g[4].loc,  0.15, 	0.001,	0.03,   50,    -50);
	Pid_increment_struct_init(&motor_pid_g[4].spd,  24.0, 	1.2,	  0.035,   9500,  -9500);
	
	Pid_increment_struct_init(&motor_pid_g[5].loc,  0.15, 	0.001,	0.03,  50,    -50);
	Pid_increment_struct_init(&motor_pid_g[5].spd,  24.0, 	1.2,	  0.035,   9500,  -9500);
	
	Pid_increment_struct_init(&motor_pid_g[6].loc,  0.15, 	0.001,	0.03,  50,    -50);
	Pid_increment_struct_init(&motor_pid_g[6].spd,  24.0, 	1.2,	  0.035,   9500,  -9500);
	
	Pid_increment_struct_init(&motor_pid_g[7].loc,  0.15, 	0.001,	0.03,  500,    -500);
	Pid_increment_struct_init(&motor_pid_g[7].spd,  24.0, 	1.2,	  0.035,   9500,  -9500);
}
/*
1.函数功能：增量式pid计算
2.入参：pid参数结构体指针，当前值，目标值
3.返回值：none
4.用法及调用要求：
5.其它：未添加调节死区，后续添加
*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target) {

	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType delta_pid_cal_result = 0;//输出量的增量
	ElemType pid_cal_result = pid_struct->last_out;//最终输出结果

	pid_struct->err = target - position;//更新本次误差

	/*计算输出量*/
	p_out = (pid_struct->Kp) * (pid_struct->err - pid_struct->err_last);
	i_out = (pid_struct->Ki) * (pid_struct->err);
	d_out = (pid_struct->Kd) * (pid_struct->err - 2 * (pid_struct->err_last) + pid_struct->err_last_last);

	/*更新上上次和上次误差*/
	pid_struct->err_last_last = pid_struct->err_last;
	pid_struct->err_last = pid_struct->err;

	/*得到最终输出结果*/
	delta_pid_cal_result = p_out + i_out + d_out;
	pid_cal_result += delta_pid_cal_result;

	/*输出限幅*/
	if (pid_cal_result > pid_struct->out_limit_up) {
		pid_cal_result = pid_struct->out_limit_up;
	}
	else if (pid_cal_result < pid_struct->out_limit_down) {
		pid_cal_result = pid_struct->out_limit_down;
	}
	
	pid_struct->last_out=pid_cal_result;
	
	pid_struct->now_out=pid_cal_result;//记录本次输出结果
	
	return pid_cal_result;
}