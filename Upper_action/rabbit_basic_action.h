#ifndef RABBIT_BASIC_ACTION
#define RABBIT_BASIC_ACTION

#include "vesc.h"
#include "usart.h"
#include "DJI_motor_contral.h"
#include "parameter_table.h"

#define lift_3508 0
#define overturn_3508 1
#define clamp_2006 2

//小兔动作的基本函数：
//发射函数
void lauch_work(int speed);
void lauch_stop(void);
//翻转函数
int overturn_up(void);
int overturn_down(void);
//夹环函数
int clamp_open(void);
int clamp_loose(void);
int clamp_close(void);
//抬升函数
int lift_work(int target_angle);
int lift_work_to_get_circle(void);
int lift_work_reset(void);

#endif
