#include "pid.h"

/**************�ڲ������뺯��begin**************/
/*����ʽpid��ʼ��*/
static void Pid_increment_struct_init(pid_incremental_struct* pid_struct, ElemType Kp, ElemType Ki, ElemType Kd, ElemType out_limit_up, ElemType out_limit_down);
/**************�ڲ������뺯��end**************/


/**************�ⲿ�ӿ�begin**************/
motor_pid_parameter motor_pid_g[8];/*���pid����*/
void Pid_parameter_init(void);/*����pid������ʼ������,�����ȵ���*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target);/*����ʽpid����*/
/**************�ⲿ�ӿ�end**************/

/*
1.�������ܣ���ʼ��һ��pid�еĸ�����
2.��Σ��ṹ��ָ�룬kp��ki��kd��������ޣ��������
3.����ֵ��none
4.�÷�������Ҫ��
5.������
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
1.�������ܣ�����pid������ʼ������
2.��Σ�none
3.����ֵ��none
4.�÷�������Ҫ����ʹ�õ��֮ǰ�����ȵ��ô˺�������pid������ʼ��
5.������
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
1.�������ܣ�����ʽpid����
2.��Σ�pid�����ṹ��ָ�룬��ǰֵ��Ŀ��ֵ
3.����ֵ��none
4.�÷�������Ҫ��
5.������δ��ӵ����������������
*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target) {

	ElemType p_out = 0;
	ElemType i_out = 0;
	ElemType d_out = 0;
	ElemType delta_pid_cal_result = 0;//�����������
	ElemType pid_cal_result = pid_struct->last_out;//����������

	pid_struct->err = target - position;//���±������

	/*���������*/
	p_out = (pid_struct->Kp) * (pid_struct->err - pid_struct->err_last);
	i_out = (pid_struct->Ki) * (pid_struct->err);
	d_out = (pid_struct->Kd) * (pid_struct->err - 2 * (pid_struct->err_last) + pid_struct->err_last_last);

	/*�������ϴκ��ϴ����*/
	pid_struct->err_last_last = pid_struct->err_last;
	pid_struct->err_last = pid_struct->err;

	/*�õ�����������*/
	delta_pid_cal_result = p_out + i_out + d_out;
	pid_cal_result += delta_pid_cal_result;

	/*����޷�*/
	if (pid_cal_result > pid_struct->out_limit_up) {
		pid_cal_result = pid_struct->out_limit_up;
	}
	else if (pid_cal_result < pid_struct->out_limit_down) {
		pid_cal_result = pid_struct->out_limit_down;
	}
	
	pid_struct->last_out=pid_cal_result;
	
	pid_struct->now_out=pid_cal_result;//��¼����������
	
	return pid_cal_result;
}