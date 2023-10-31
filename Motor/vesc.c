#include "vesc.h"
#include "can.h"
#include "can_database.h"

/**************�ڲ��궨����������begin**************/

/**************�ڲ��궨����������end**************/

/**************�ڲ������뺯��begin**************/
union s32_to_u8 vesc_content_transform[vesc_motor_nums]={0};//�洢��ת��vesc���������
static int vesc_motor_poles_s[vesc_motor_nums]={6,6};//����ļ�����,������±�Ϊ1��ʼ��Ӧ1�ŵ�����±�Ϊ0����������
static int motor_speed_s[vesc_motor_nums]={0};//���͵���ٶȣ��±�Ϊ���ID
static void Write_database_vesc_extid(uint32_t id_num);//��չID���ķ��ͺ���
/**************�ڲ������뺯��end**************/


/**************�ⲿ�ӿ�begin**************/
void Change_vesc_speed(int motor_id,int target_spd);//���Ķ�Ӧid��vesc�ٶ�
void Com2vesc(uint32_t motor_id);//���1ms�ڱ�����һ�Σ��̶����ͱ���
void Vesc_speed_contral_init(void);//����vesc��ʼ�ٶ���Ϊ0
/**************�ⲿ�ӿ�end**************/

/*
1.�������ܣ�����vesc����ٶ�
2.��Σ����ID+Ŀ���ٶ�
3.����ֵ����
4.�÷�������Ҫ��
5.������
*/
void Change_vesc_speed(int motor_id,int target_spd){
	 motor_speed_s[motor_id]=target_spd;
}

/*
1.�������ܣ���䲢���� vesc �ٶ� ���Ʊ��ĵ�id��content
2.��Σ����ID
3.����ֵ����
4.�÷�������Ҫ�󣺼��1ms���ڵ���һ��
5.������
*/
void Com2vesc(uint32_t motor_id){
	uint32_t vesc_speed_id=motor_id+(CAN_PACKET_SET_RPM<<8);
	uint32_t Erpm= motor_speed_s[motor_id] * vesc_motor_poles_s[motor_id];//ת��Ϊ��Ƕ�
	vesc_content_transform[motor_id].u8_data[0]=Erpm>>24;
	vesc_content_transform[motor_id].u8_data[1]=Erpm>>16;
	vesc_content_transform[motor_id].u8_data[2]=Erpm>>8;
	vesc_content_transform[motor_id].u8_data[3]=Erpm;
	Write_database_vesc_extid(vesc_speed_id);
}

/*
1.�������ܣ����е����ʼ�ٶ���Ϊ0
2.��Σ���
3.����ֵ����
4.�÷�������Ҫ��
5.������
*/
void Vesc_speed_contral_init(void){
	for (int i=0;i<vesc_motor_nums;i++){
		motor_speed_s[i]=0;
	}
}

/*
1.�������ܣ����vesc���Ʊ��Ĳ�����
2.��Σ����ID
3.����ֵ����
4.�÷�������Ҫ��
5.������ע��������Լ��������͵Ĵ���
*/
static void Write_database_vesc_extid(uint32_t id_num){
	CAN_TxHeaderTypeDef TxMessage;
	int sendnum=0;//���ʹ�������ֹ����

	if((hash_table[id_num] >= can_data_num_g)){
		return;
	}//������
	
	TxMessage.StdId=0;
	TxMessage.ExtId=can_database_g[hash_table[id_num]].Data_ID;
	TxMessage.IDE=CAN_ID_EXT;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=can_database_g[hash_table[id_num]].Data_length;
	TxMessage.TransmitGlobalTime=ENABLE;
	
	uint32_t pTxMailbox=0;
	if(can_database_g[hash_table[id_num]].Channel==1){
		while(HAL_CAN_AddTxMessage(&hcan1, &TxMessage, can_database_g[hash_table[id_num]].Data_ptr,  &pTxMailbox) != HAL_OK){
			sendnum++;
			if(sendnum==10){
				break;//��������
			}
    }	
	}else{
		while(HAL_CAN_AddTxMessage(&hcan2, &TxMessage, can_database_g[hash_table[id_num]].Data_ptr,  &pTxMailbox) != HAL_OK){
			sendnum++;
			if(sendnum==10){
				break;//��������
			}
    }	
	}
}

#if 0

void set_vesc_position(uint32_t ID,int32_t location){
	uint32_t vesc_position_id=ID+(CAN_PACKET_SET_POS<<8);
	location*=1000000;
	vesc_content_transform[ID].u8_data[0]=location>>24;
	vesc_content_transform[ID].u8_data[1]=location>>16;
	vesc_content_transform[ID].u8_data[2]=location>>8;
	vesc_content_transform[ID].u8_data[3]=location;
	Write_database_vesc_extid(vesc_position_id);
}

void set_vesc_current(uint32_t ID,int32_t current){
	uint32_t vesc_current_id=ID+(CAN_PACKET_SET_CURRENT<<8);
	current*=1000;
	vesc_content_transform[ID].u8_data[0]=current>>24;
	vesc_content_transform[ID].u8_data[1]=current>>16;
	vesc_content_transform[ID].u8_data[2]=current>>8;
	vesc_content_transform[ID].u8_data[3]=current;
	Write_database_vesc_extid(vesc_current_id);
}

#endif

