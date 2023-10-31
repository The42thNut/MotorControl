#ifndef BASIC_H
#define BASIC_H

/**************************
说明：均为外部接口，存放常用数学函数或结构体以及常量
**************************/

int Basic_int_abs(int x);
//pi
extern const float basic_pi;

//x方向和y方向参数
typedef struct{
	
}basic_vector_x_y; 

//比较多个数的大小
void Basic_compare_nums();

//得到坐标相对于y轴的角度大小
float Basic_arctan2y();

//将角度制转换为弧度制并限制在-pi到pi的范围内 
void Basic_degree2rad();

//限制数据的上限和下限
void Basic_limit();// 

//在实际情况下判断两个数是否相等，差值小于某个阈值 
int Basic_equality_judge();

#endif
