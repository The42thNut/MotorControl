#ifndef RABBIT_ACTION
#define RABBIT_ACTION

extern int get_circle_begin;//取环开始标志位
extern int clamp_situation_ok_flag;//环进入取环范围的标志位
extern int lift_target_total_angle;

int get_circle(void);
int overturn2zero(void);
void get_circle_all_action();
void lauch_lift_work(int lauch_speed,int target_angle);//发射和抬升工作函数
#endif