#include "dji_motor.h"
#include "main.h"
#include "can_database.h"
#include "can.h"
#include "pid.h"
#include "basic.h"
#include "usart.h"

//3508电流范围   -16384-16384
//2006电流范围   -10000-10000

/**************内部宏定义与重命名begin**************/

//#define ALL_Send_Flag //收到所有电机报文后发送一条报文的标志

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
static int set_spd_s[8]={0,0,0,0,0,0,0,0};
static int set_loc_s[8]={0,0,0,0,0,0,0,0};
static int mode_s[8]={	LOC_MODE,
												LOC_MODE,
												LOC_MODE,
												LOC_MODE,
												LOC_MODE,
												LOC_MODE,
												LOC_MODE,
												LOC_MODE  };
static motor_measure_t motor_inf[8]={0};/*3508电机参数*/
static void Get_total_angle(motor_measure_t *p);
static void Can_cmd_first_four_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
static void Can_cmd_last_four_motor(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
/**************内部变量与函数end**************/


/**************外部接口begin**************/
void Dji_motor_control(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]);
void Change_dji_speed(int motor_id,int target_spd);
void Change_dji_loc(int motor_id,int target_loc);
motor_measure_t Get_dji_information(int motor_id);
/**************外部接口end**************/

/*
1.函数功能：请求dji电机信息，包括速度、位置等
2.入参：电机ID
3.返回值：电机信息结构体
4.用法及调用要求：
5.其它：
*/
motor_measure_t Get_dji_information(int motor_id){
	return motor_inf[motor_id];
}
/*
1.函数功能：设定dji电机的速度大小
2.入参：电机ID，speed（RPM）
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_speed(int motor_id,int target_spd){
	set_spd_s[motor_id]=target_spd;
}
/*
1.函数功能：设定dji电机的位置
2.入参：电机ID，整数值，编码器相关
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_loc(int motor_id,int target_loc){
	set_loc_s[motor_id]=target_loc;
}
/*
1.函数功能：得到dji电调的电机信息返回值
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：宏函数
*/
#define Get_motor_measure(ptr, data)                                    \
    {                                                                  \
        (ptr)->last_angle = (ptr)->angle;                                   \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#ifdef ALL_Send_Flag
		int can1_send_flag[8]={0};//收到所有电机报文后发送一条报文的标志
#endif
/*
1.函数功能：得到dji电机的总转程
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：
*/
static void Get_total_angle(motor_measure_t *p){
		int res1, res2, delta;
		if(p->angle < p->last_angle){			//可能的情况
			res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
			res2 = p->angle - p->last_angle;				//反转	delta=-
		}else{	//angle > last
			res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
			res2 = p->angle - p->last_angle;				//正转	delta +
		}
		//不管正反转，肯定是转的角度小的那个是真的
		if(Basic_int_abs(res1)<Basic_int_abs(res2))
			delta = res1;
		else
			delta = res2;

		p->total_angle += delta;
		p->last_angle = p->angle;
}
/*
1.函数功能：发送ID前四的电机的电流值
2.入参：
3.返回值：无
4.用法及调用要求：注意修改发送通道为CAN1或CAN2
5.其它：
*/
static void Can_cmd_first_four_motor(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
	  static CAN_TxHeaderTypeDef  first_four_motor_tx_message;
		static uint8_t              first_four_motor_can_send_data[8];
    uint32_t send_mail_box;
    first_four_motor_tx_message.StdId = CAN_FIRST_FOUR_MOTOR_ALL_ID;
    first_four_motor_tx_message.IDE = CAN_ID_STD;
    first_four_motor_tx_message.RTR = CAN_RTR_DATA;
    first_four_motor_tx_message.DLC = 0x08;
    first_four_motor_can_send_data[0] = motor1 >> 8;
    first_four_motor_can_send_data[1] = motor1;
    first_four_motor_can_send_data[2] = motor2 >> 8;
    first_four_motor_can_send_data[3] = motor2;
    first_four_motor_can_send_data[4] = motor3 >> 8;
    first_four_motor_can_send_data[5] = motor3;
    first_four_motor_can_send_data[6] = motor4 >> 8;
    first_four_motor_can_send_data[7] = motor4;
	
    HAL_CAN_AddTxMessage(&hcan1, &first_four_motor_tx_message, first_four_motor_can_send_data, &send_mail_box);
}
/*
1.函数功能：发送ID后四的电机的电流值
2.入参：
3.返回值：无
4.用法及调用要求：注意修改发送通道为CAN1或CAN2
5.其它：
*/	
static void Can_cmd_last_four_motor(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8){
		static CAN_TxHeaderTypeDef  last_four_motor_tx_message;
		static uint8_t              last_four_motor_can_send_data[8];
    uint32_t send_mail_box;
    last_four_motor_tx_message.StdId = CAN_LAST_FOUR_MOTOR_ALL_ID;
    last_four_motor_tx_message.IDE = CAN_ID_STD;
    last_four_motor_tx_message.RTR = CAN_RTR_DATA;
    last_four_motor_tx_message.DLC = 0x08;
    last_four_motor_can_send_data[0] = (motor5 >> 8);
    last_four_motor_can_send_data[1] = motor5;
    last_four_motor_can_send_data[2] = (motor6 >> 8);
    last_four_motor_can_send_data[3] = motor6;
    last_four_motor_can_send_data[4] = (motor7 >> 8);
    last_four_motor_can_send_data[5] = motor7;
    last_four_motor_can_send_data[6] = (motor8 >> 8);
    last_four_motor_can_send_data[7] = motor8;
    HAL_CAN_AddTxMessage(&hcan1, &last_four_motor_tx_message, last_four_motor_can_send_data, &send_mail_box);
}
/*
1.函数功能：pid计算出电机此次应当输出的电流值并发送
2.入参：报文的header，报文数据域内容
3.返回值：无
4.用法及调用要求：在dji电机的CAN的接收回调里调用此函数
5.其它：
*/
void Dji_motor_control(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]){
	  		static uint8_t i = 0;           
				i = rx_header.StdId - CAN_3508_M1_ID;
				#ifdef ALL_Send_Flag
				can1_send_flag[i]=1;
				cnt[i]++;
				#endif
				Get_motor_measure(&motor_inf[i], rx_data);//得到电机信息
				if (motor_inf[i].first == 0){
						motor_inf[i].first = 1;
						motor_inf[i].last_angle=motor_inf[i].angle;
				}
				Get_total_angle(&motor_inf[i]);
				if(mode_s[i]==LOC_MODE){
					Pid_incremental_cal(&motor_pid_g[i].loc,motor_inf[i].total_angle,set_loc_s[i]);//利用电机反馈的速度和位置信息做pid计算
					Pid_incremental_cal(&motor_pid_g[i].spd,motor_inf[i].speed_rpm,motor_pid_g[i].loc.now_out);
				}
				else if(mode_s[i]==SPEED_MODE){
					Pid_incremental_cal(&motor_pid_g[i].spd,motor_inf[i].speed_rpm,set_spd_s[i]);
				}
				#ifdef ALL_Send_Flag
					if(can1_send_flag[0] && can1_send_flag[1] && can1_send_flag[2] &&can1_send_flag[3] && can1_send_flag[4] &&can1_send_flag[5] &&can1_send_flag[6]){
						Can_cmd_first_four_motor((int16_t)motor_pid_g[0].spd.now_out,   //将PID的计算结果通过CAN发到电机
						(int16_t)motor_pid_g[1].spd.now_out,
						(int16_t)motor_pid_g[2].spd.now_out,
						(int16_t)motor_pid_g[3].spd.now_out);
						can1_send_flag[0]=0;can1_send_flag[1] 0;can1_send_flag[2]=0;can1_send_flag[3]=0;
						Can_cmd_last_four_motor((int16_t)motor_pid_g[4].spd.now_out,   //将PID的计算结果通过CAN发送到电机
						(int16_t)motor_pid_g[5].spd.now_out,
						(int16_t)motor_pid_g[6].spd.now_out,
						(int16_t)motor_pid_g[7].spd.now_out);
						can1_send_flag[4]=0;can1_send_flag[5]=0;can1_send_flag[6]=0;
					}
				#else

				Can_cmd_first_four_motor((int16_t)motor_pid_g[0].spd.now_out,   //将PID的计算结果通过CAN发到电机
					(int16_t)motor_pid_g[1].spd.now_out,
					(int16_t)motor_pid_g[2].spd.now_out,
					(int16_t)motor_pid_g[3].spd.now_out);
				Can_cmd_last_four_motor((int16_t)motor_pid_g[4].spd.now_out,   //将PID的计算结果通过CAN发送到电机
						(int16_t)motor_pid_g[5].spd.now_out,
						(int16_t)motor_pid_g[6].spd.now_out,
						(int16_t)motor_pid_g[7].spd.now_out);
				#endif
}
