#include "math.h"
#include "stdio.h"
#include "control_motor.h"
#include "dji_motor.h"
#include "basic.h"

//云台0
float theta1;
float h=120.0;
//第一机械臂1
float theta2;
float l1=360.0;
//第二机械臂2
float theta3;
float l2=220.0;
 //电机信息结构体
 motor_measure_t ptr_measure;
//判断电机是否转动完成
 int InRound;
//定义常数   
const float  pai_s=3.1415;


/*
1.函数功能：生成贝塞尔曲线上点
2.入参：
3.返回值：
4.用法及调用要求：
5.其它：
*/
void Calculate_bezierPoint(BezierControlPoint p0, BezierControlPoint p1, BezierControlPoint p2, BezierControlPoint p3, float t, float* x, float* y, float* z) {
    float u = 1 - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;

    *x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    *y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    *z = uuu * p0.z + 3 * uu * t * p1.z + 3 * u * tt * p2.z + ttt * p3.z;
}

/*
1.函数功能：将指定坐标转化为机械臂三个电机应转的角度，并调用转动函数使电机移动
2.入参：贝塞尔曲线得到的轨迹取的坐标/固定坐标
3.返回值：1 能到达该点 0 不能到达该点
4.用法及调用要求：
5.其它：
*/
int Switch_Move(float x, float y, float z) {
    // 计算关节1角度
    theta1 = atan2(y, x);

    // 计算关节2角度
    theta2=(l1*l1+x*x+y*y+(z-h)*(z-h)-l2*l2)/(2*l1*sqrt(x*x+y*y+(z-h)*(z-h)));
    if (theta2 >= -1 && theta2 <= 1) {
        theta2 = acos(theta2)+atan2((z-h),sqrt(x*x+y*y));
    } else {
        return 0; // 无解
    }
    
    // 计算关节3角度
		theta3=(l1*l1+l2*l2-x*x-y*y-(z-h)*(z-h))/(2*l1*l2);
     if (theta3 >= -1 && theta3 <= 1) {
        theta3 = acos(theta3);
    } else {
        return 0; // 无解
    }
    theta1=theta1*180/pai_s;
    theta2=theta2*180/pai_s;
    theta3=theta3*180/pai_s;
    return 1; // 有解
}
/*
1.函数功能：使电机以位置模式转过指定角度
2.入参：电机ID,转动角度
3.返回值：无
4.用法及调用要求：
5.其它：判断是否转完，转完后打印目前电机角度值
*/
void Set_motor_angle(int Motor_ID,float Target_angle)
{
		float nowangle;
    Change_dji_loc(Motor_ID,Target_angle);
    while(abs(Get_dji_information(Motor_ID).total_angle-Target_angle)>50)
		{
			ptr_measure=Get_dji_information(Motor_ID);
			nowangle=ptr_measure.total_angle;}
			printf("nowangle: %d=%lf \n",Motor_ID,nowangle);
}
/*
1.函数功能：控制电机运动
2.入参：起始坐标 终点坐标
3.返回值：
4.用法及调用要求：
5.其它：
*/
void Motor_control(float x_initial,float y_initial,float z_initial,float x_target,float y_target,float z_target)
{
	// 定义贝塞尔曲线的控制点
		BezierControlPoint p0 = {x_initial,y_initial,z_initial};
    BezierControlPoint p1 = {10.0, 20.0,200.0};
    BezierControlPoint p2 = {10.0, 20.0, 200.0};
    BezierControlPoint p3 = {x_target,y_target,z_target};

    // 沿贝塞尔曲线生成轨迹点
    int numPoints = 5; // 轨迹上的点数
    for (int i = 0; i <=numPoints; i++) {
        float t = (float)i / numPoints;
        float x, y, z;
        Calculate_bezierPoint(p0, p1, p2, p3, t, &x, &y, &z);
        // 计算逆运动学，获取关节角度
       
        if (Switch_Move(x, y, z)) {
          //角度转换：将算出的角度乘以对应的传动比后转化为电机的位置信息
						printf("realangle: a=%lf b=%lf c=%lf\n",theta1 ,theta2 ,theta3 );
						theta1=theta1*72/20/360*8191*20;
						theta2=theta2*30/18/360*8191*20;
						theta3=theta3/360*8191*20;
					//控制运动 
					printf("angle: theta1=%lf , theta2=%lf , theta3=%lf \n",theta1 ,theta2 ,theta3 );
					Set_motor_angle(1,theta1);
					Set_motor_angle(2,theta2);
					Set_motor_angle(4,theta3);
        }
        
    }
}