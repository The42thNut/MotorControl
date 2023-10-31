#ifndef HHH_H
#define HHH_H

#include "dji_motor.h"
// 定义贝塞尔曲线的控制点
typedef struct {
    double x;
    double y;
    double z;
} BezierControlPoint;


void Calculate_bezierPoint(BezierControlPoint p0, BezierControlPoint p1, BezierControlPoint p2, BezierControlPoint p3, float t, float* x, float* y, float* z);
int Switch_Move(float x, float y, float z);
void Motor_control(float x_initial,float y_initial,float z_initial,float x_target,float y_target,float z_target);

#endif