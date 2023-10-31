#ifndef RABBIT_BASIC_ACTION
#define RABBIT_BASIC_ACTION

#include "vesc.h"
#include "usart.h"
#include "DJI_motor_contral.h"
#include "parameter_table.h"

#define lift_3508 0
#define overturn_3508 1
#define clamp_2006 2

//С�ö����Ļ���������
//���亯��
void lauch_work(int speed);
void lauch_stop(void);
//��ת����
int overturn_up(void);
int overturn_down(void);
//�л�����
int clamp_open(void);
int clamp_loose(void);
int clamp_close(void);
//̧������
int lift_work(int target_angle);
int lift_work_to_get_circle(void);
int lift_work_reset(void);

#endif
