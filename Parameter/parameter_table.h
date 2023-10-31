#ifndef PARAMETER_TABLE_H
#define PARAMETER_TABLE_H

#define column_num 4//柱子类型数
#define point_num 5//柱子的最大距离数

#define overturn_angle -139500 //翻转3508的转程，上电时在机械限位处 
#define clamp_close_distance 85294   //夹紧2006的转程，上电时在最大处
#define clamp_loose_distance 55000
#define lift_get_circle_angle -12000//-13800 //抬升配合翻转取环时，抬升的转程，上电在机械限位处
#define motor_angle_error 1000

typedef struct {
	int distance;//测试距离用，以此筛选出最优距离,单位是mm
	int launch_speed;//5065发射电机速度
	float elevat_angle;//抬升仰角
	float hit_rate;//测试用，以此筛选出最优距离
} point_infm;//点位信息

typedef struct {
	point_infm point[point_num];
} map_parameter;//柱子类型，以柱子的高度进行分类


extern int column_type;
extern int test_distance;
extern map_parameter column_infm[column_num];

#endif 
