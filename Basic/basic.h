#ifndef BASIC_H
#define BASIC_H

/**************************
˵������Ϊ�ⲿ�ӿڣ���ų�����ѧ������ṹ���Լ�����
**************************/

int Basic_int_abs(int x);
//pi
extern const float basic_pi;

//x�����y�������
typedef struct{
	
}basic_vector_x_y; 

//�Ƚ϶�����Ĵ�С
void Basic_compare_nums();

//�õ����������y��ĽǶȴ�С
float Basic_arctan2y();

//���Ƕ���ת��Ϊ�����Ʋ�������-pi��pi�ķ�Χ�� 
void Basic_degree2rad();

//�������ݵ����޺�����
void Basic_limit();// 

//��ʵ��������ж��������Ƿ���ȣ���ֵС��ĳ����ֵ 
int Basic_equality_judge();

#endif
