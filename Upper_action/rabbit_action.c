#include "rabbit_action.h"
#include "rabbit_basic_action.h"
#include "main.h"
#include "com_chassis.h"
#include "vesc.h"

int get_circle_begin=0;//ȡ����ʼ��־λ
int clamp_situation_ok_flag=0;//������ȡ����Χ�ı�־λ
int lauch_circle_begin=0;//���뷢��״̬ʱ̧����ʼ������־λ

int lift_target_total_angle=0;//̧���Ƕ�

//С�õ�������������

//����ȡ������
int get_circle(void) //ͬʱ��ת���ſ�ȡ����̧����λ�ŷ���1�����򷵻�0
{
	lauch_stop();//����ֹͣ
	int res=0;
	res = overturn_down();
	res *= clamp_open();
	res *= lift_work_to_get_circle();
	return res;
}

//̧��3508�ͷ�ת3508ͬʱ��λ
int overturn2zero(void)//ͬʱ��λ������λ�󷵻�1
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
		//���������Ƿ���ɵı�־λ��Ϊ1��ʾ��δ��ɣ�Ϊ0Ϊ�����
		static uint8_t get_circle_ok = 1, clamp_close_ok = 1, overturn2zero_ok = 1, clamp_loose_ok = 1;
		if (get_circle_ok && get_circle() == 0) break;
		com_finish_invert();//�����̷���Ϣ���ϲ㷭ת���
		//��ת��ȡ����̧�����ѵ�λ
		get_circle_ok = 0;
		if (clamp_situation_ok_flag)
		{
			if (clamp_close_ok && clamp_close() == 0)break;
			clamp_close_ok = 0;
			com_close_finish();//�����̷���Ϣ���ϲ�л����
			if (overturn2zero_ok && overturn2zero() == 0)break;
			overturn2zero_ok = 0;
			HAL_Delay(400);
			if (clamp_loose_ok && clamp_loose() == 0)break;
			clamp_loose_ok = 0;

			//����ȡ�����������
			clamp_situation_ok_flag = 0;
			get_circle_begin = 0;
			get_circle_ok = clamp_close_ok = overturn2zero_ok = clamp_loose_ok = 1;
		}
	}
}
uint32_t push_count_time1=0;
uint32_t push_count_time2=0;

void lauch_lift_work(int lauch_speed,int target_angle){//�����̧���Լ��ƻ���������
	lauch_work(lauch_speed);//���乤��
	while(lauch_circle_begin){
		static uint8_t lift_work_ok = 1;
		static uint8_t first_push_count_flag=1;
		if(lift_work_ok && lift_work(target_angle)==0) break;//̧������
		lift_work_ok=0;
		push_count_time1=HAL_GetTick();
		if(first_push_count_flag){
			push_count_time2=push_count_time1;
			first_push_count_flag=0;
		}
		HAL_GPIO_WritePin(GPIOD, cylinder_contral_first_Pin, GPIO_PIN_SET);//����
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

