#include "math.h"
#include "stdio.h"
#include "control_motor.h"
#include "dji_motor.h"
#include "basic.h"

//��̨0
float theta1;
float h=120.0;
//��һ��е��1
float theta2;
float l1=360.0;
//�ڶ���е��2
float theta3;
float l2=220.0;
 //�����Ϣ�ṹ��
 motor_measure_t ptr_measure;
//�жϵ���Ƿ�ת�����
 int InRound;
//���峣��   
const float  pai_s=3.1415;


/*
1.�������ܣ����ɱ����������ϵ�
2.��Σ�
3.����ֵ��
4.�÷�������Ҫ��
5.������
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
1.�������ܣ���ָ������ת��Ϊ��е���������Ӧת�ĽǶȣ�������ת������ʹ����ƶ�
2.��Σ����������ߵõ��Ĺ켣ȡ������/�̶�����
3.����ֵ��1 �ܵ���õ� 0 ���ܵ���õ�
4.�÷�������Ҫ��
5.������
*/
int Switch_Move(float x, float y, float z) {
    // ����ؽ�1�Ƕ�
    theta1 = atan2(y, x);

    // ����ؽ�2�Ƕ�
    theta2=(l1*l1+x*x+y*y+(z-h)*(z-h)-l2*l2)/(2*l1*sqrt(x*x+y*y+(z-h)*(z-h)));
    if (theta2 >= -1 && theta2 <= 1) {
        theta2 = acos(theta2)+atan2((z-h),sqrt(x*x+y*y));
    } else {
        return 0; // �޽�
    }
    
    // ����ؽ�3�Ƕ�
		theta3=(l1*l1+l2*l2-x*x-y*y-(z-h)*(z-h))/(2*l1*l2);
     if (theta3 >= -1 && theta3 <= 1) {
        theta3 = acos(theta3);
    } else {
        return 0; // �޽�
    }
    theta1=theta1*180/pai_s;
    theta2=theta2*180/pai_s;
    theta3=theta3*180/pai_s;
    return 1; // �н�
}
/*
1.�������ܣ�ʹ�����λ��ģʽת��ָ���Ƕ�
2.��Σ����ID,ת���Ƕ�
3.����ֵ����
4.�÷�������Ҫ��
5.�������ж��Ƿ�ת�꣬ת����ӡĿǰ����Ƕ�ֵ
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
1.�������ܣ����Ƶ���˶�
2.��Σ���ʼ���� �յ�����
3.����ֵ��
4.�÷�������Ҫ��
5.������
*/
void Motor_control(float x_initial,float y_initial,float z_initial,float x_target,float y_target,float z_target)
{
	// ���屴�������ߵĿ��Ƶ�
		BezierControlPoint p0 = {x_initial,y_initial,z_initial};
    BezierControlPoint p1 = {10.0, 20.0,200.0};
    BezierControlPoint p2 = {10.0, 20.0, 200.0};
    BezierControlPoint p3 = {x_target,y_target,z_target};

    // �ر������������ɹ켣��
    int numPoints = 5; // �켣�ϵĵ���
    for (int i = 0; i <=numPoints; i++) {
        float t = (float)i / numPoints;
        float x, y, z;
        Calculate_bezierPoint(p0, p1, p2, p3, t, &x, &y, &z);
        // �������˶�ѧ����ȡ�ؽڽǶ�
       
        if (Switch_Move(x, y, z)) {
          //�Ƕ�ת����������ĽǶȳ��Զ�Ӧ�Ĵ����Ⱥ�ת��Ϊ�����λ����Ϣ
						printf("realangle: a=%lf b=%lf c=%lf\n",theta1 ,theta2 ,theta3 );
						theta1=theta1*72/20/360*8191*20;
						theta2=theta2*30/18/360*8191*20;
						theta3=theta3/360*8191*20;
					//�����˶� 
					printf("angle: theta1=%lf , theta2=%lf , theta3=%lf \n",theta1 ,theta2 ,theta3 );
					Set_motor_angle(1,theta1);
					Set_motor_angle(2,theta2);
					Set_motor_angle(4,theta3);
        }
        
    }
}