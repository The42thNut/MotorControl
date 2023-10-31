#ifndef PID_H
#define PID_H

typedef float ElemType;

typedef struct {
	ElemType Kp;
	ElemType Ki;
	ElemType Kd;

	ElemType last_out;//上次输出

	ElemType err;//本次误差
	ElemType err_last;//上次误差
	ElemType err_last_last;//上上次误差

	ElemType out_limit_up;//最大输出限幅
	ElemType out_limit_down;//最小输出限幅
	
	ElemType now_out;//本次输出值，正常pid不需要，这里为了方便添加
}pid_incremental_struct;

typedef struct {
	pid_incremental_struct spd;
	pid_incremental_struct loc;	
}motor_pid_parameter;

/**************Public_begin**************/
extern motor_pid_parameter motor_pid_g[8];
void Pid_parameter_init(void);/*各套pid参数初始化参数,在使用电机前必须先进行调用*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target);/*增量式pid计算*/
/**************Public_end**************/

#endif