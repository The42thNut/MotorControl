#ifndef RABBIT_ACTION
#define RABBIT_ACTION

extern int get_circle_begin;//ȡ����ʼ��־λ
extern int clamp_situation_ok_flag;//������ȡ����Χ�ı�־λ
extern int lift_target_total_angle;

int get_circle(void);
int overturn2zero(void);
void get_circle_all_action();
void lauch_lift_work(int lauch_speed,int target_angle);//�����̧����������
#endif