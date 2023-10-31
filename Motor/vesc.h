#ifndef vesc_h
#define vesc_h
#include "stm32f4xx.h"


#define vesc_motor_nums 2//���Ƶĵ������

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
} CAN_PACKET_ID;

union s32_to_u8{     //һ��32λ����תΪ4��8λ���ݽ���can�ķ���
	uint32_t s32_data;
	uint8_t u8_data[4];
};


extern union s32_to_u8 vesc_content_transform[vesc_motor_nums];


/**************�ⲿ�ӿ�begin**************/
void Com2vesc(uint32_t motor_id);//���1ms�ڱ�����һ�Σ��̶����ͱ���
void Change_vesc_speed(int motor_id,int target_spd);//���Ķ�Ӧid��vesc�ٶ�
void Vesc_speed_contral_init(void);//����vesc��ʼ�ٶ���Ϊ0
/**************�ⲿ�ӿ�end**************/

#endif
