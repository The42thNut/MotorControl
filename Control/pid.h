#ifndef PID_H
#define PID_H

typedef float ElemType;

typedef struct {
	ElemType Kp;
	ElemType Ki;
	ElemType Kd;

	ElemType last_out;//�ϴ����

	ElemType err;//�������
	ElemType err_last;//�ϴ����
	ElemType err_last_last;//���ϴ����

	ElemType out_limit_up;//�������޷�
	ElemType out_limit_down;//��С����޷�
	
	ElemType now_out;//�������ֵ������pid����Ҫ������Ϊ�˷������
}pid_incremental_struct;

typedef struct {
	pid_incremental_struct spd;
	pid_incremental_struct loc;	
}motor_pid_parameter;

/**************Public_begin**************/
extern motor_pid_parameter motor_pid_g[8];
void Pid_parameter_init(void);/*����pid������ʼ������,��ʹ�õ��ǰ�����Ƚ��е���*/
ElemType Pid_incremental_cal(pid_incremental_struct* pid_struct, ElemType position, ElemType target);/*����ʽpid����*/
/**************Public_end**************/

#endif