#ifndef PARAMETER_TABLE_H
#define PARAMETER_TABLE_H

#define column_num 4//����������
#define point_num 5//���ӵ���������

#define overturn_angle -139500 //��ת3508��ת�̣��ϵ�ʱ�ڻ�е��λ�� 
#define clamp_close_distance 85294   //�н�2006��ת�̣��ϵ�ʱ�����
#define clamp_loose_distance 55000
#define lift_get_circle_angle -12000//-13800 //̧����Ϸ�תȡ��ʱ��̧����ת�̣��ϵ��ڻ�е��λ��
#define motor_angle_error 1000

typedef struct {
	int distance;//���Ծ����ã��Դ�ɸѡ�����ž���,��λ��mm
	int launch_speed;//5065�������ٶ�
	float elevat_angle;//̧������
	float hit_rate;//�����ã��Դ�ɸѡ�����ž���
} point_infm;//��λ��Ϣ

typedef struct {
	point_infm point[point_num];
} map_parameter;//�������ͣ������ӵĸ߶Ƚ��з���


extern int column_type;
extern int test_distance;
extern map_parameter column_infm[column_num];

#endif 
