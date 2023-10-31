#ifndef vesc_h
#define vesc_h
#include "stm32f4xx.h"


#define vesc_motor_nums 2//控制的电机总数

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
} CAN_PACKET_ID;

union s32_to_u8{     //一个32位数据转为4个8位数据进行can的发送
	uint32_t s32_data;
	uint8_t u8_data[4];
};


extern union s32_to_u8 vesc_content_transform[vesc_motor_nums];


/**************外部接口begin**************/
void Com2vesc(uint32_t motor_id);//间隔1ms内被调用一次，固定发送报文
void Change_vesc_speed(int motor_id,int target_spd);//更改对应id的vesc速度
void Vesc_speed_contral_init(void);//所有vesc初始速度设为0
/**************外部接口end**************/

#endif
