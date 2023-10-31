#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

/**********************************************
�����ļ���basic.c�е���ѧ����
					pid.c�е�pid������pid����
					can_database.c�еĻص�
**********************************************/

#include "main.h"

#define SPEED_MODE 0
#define LOC_MODE 1

typedef struct{
		uint16_t angle;
		int16_t speed_rpm;
		int16_t given_current;
		uint8_t temperate;
		int16_t last_angle;
		uint16_t	offset_angle;
		int32_t		round_cnt;
		int32_t		total_angle;
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
		uint16_t  first  ; //�����жϵ���Ƿ�Ϊ��һ�η����ź�
} motor_measure_t;

/* CAN send and receive ID */
typedef enum{
	 CAN_FIRST_FOUR_MOTOR_ALL_ID = 0x200,//djiϵ�е����
	 CAN_LAST_FOUR_MOTOR_ALL_ID = 0x1FF,
	
	 CAN_3508_M1_ID = 0x201,
	 CAN_3508_M2_ID = 0x202,
	 CAN_3508_M3_ID = 0x203,
	 CAN_3508_M4_ID = 0x204,
	 CAN_3508_M5_ID = 0x205,
	 CAN_3508_M6_ID = 0x206,
	 CAN_3508_M7_ID = 0x207,
	 CAN_3508_M8_ID = 0x208,
} can_msg_id_e;

/**************Public����begin**************/
void Dji_motor_control(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]);
void Change_dji_speed(int motor_id,int target_spd);
void Change_dji_loc(int motor_id,int target_loc);
motor_measure_t Get_dji_information(int motor_id);
/**************Public-end**************/

#endif
