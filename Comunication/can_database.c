/*
����vesc��odrive�Լ�ͨ����CAN�ź�
1.
2.
3.
*/
#include "can.h"
#include "can_database.h"
#include "vesc.h"
#include "dji_motor.h"
#include "stdio.h"
#include "usart.h"

/**************�ڲ��궨����������begin**************/

/**************�ڲ��궨����������end**************/

/**************�ڲ������뺯��begin**************/
static void Can_filter_init(void);
/**************�ڲ������뺯��end**************/


/**************�ⲿ�ӿ�begin**************/
uint8_t can_data_num_g=0;//�ṹ������can_database_g�Ĵ�С����Hash_table_init(void)�и���
Can_Data can_database_g[]={//ͨ�ű�odrive��vesc��������¼�IDʱ����ID_NUMDEF�ж�����ӦID������
    //Data_type            Data_ID             *Data_ptr                                   Data_length   *MenuFunc   Channel       Fifo_num  
		//{WRITE_ONLY,      vesc_motor1,         (uint8_t*)(&vesc_content_transform[1].u8_data),       4,          NULL,        2,      CAN_FILTER_FIFO0},
		//{WRITE_ONLY,      vesc_motor2,         (uint8_t*)(&vesc_content_transform[2].u8_data),       4,          NULL,        2,      CAN_FILTER_FIFO0},   
};
uint16_t hash_table[1000]={999};
void Hash_table_init(void);
void Can_start_work(void);
/**************�ⲿ�ӿ�end**************/

/*
1.�������ܣ���ʼ��ͨ��hash��
2.��Σ�none
3.����ֵ��none
4.�÷�������Ҫ����ʹ��vesc��odrive֮ǰ���ô˺���
5.������
*/
void Hash_table_init(void){ 
	int i;
	can_data_num_g = sizeof(can_database_g) / sizeof(can_database_g[0]);
	for(i=0;i<1000;i++){
		hash_table[i] = 999;
	}
	for(i=0;i<can_data_num_g;i++){
		hash_table[can_database_g[i].Data_ID] = i;
	}
}

/*
1.�������ܣ���ʼ��CAN������
2.��Σ�none
3.����ֵ��none
4.�÷�������Ҫ��
5.������
*/
static void Can_filter_init(void){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
	  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
}	

/*
1.�������ܣ�����CAN����������FIFO
2.��Σ�none
3.����ֵ��none
4.�÷�������Ҫ��
5.������
*/
void Can_start_work(void){
		Can_filter_init();
	  HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//dji���
{
	if(hcan==&hcan1){
		static int cnt[8]={0};
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId){
			case CAN_3508_M1_ID:	
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:{
					Dji_motor_control(rx_header,rx_data);
					break;
			}
			default:
				break;
	  }
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan==&hcan2){
		static int cnt[8]={0};
		CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
		switch (rx_header.StdId){
			case CAN_3508_M1_ID:	
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			case CAN_3508_M8_ID:{
					Dji_motor_control(rx_header,rx_data);
					break;
			}
			default:
				break;
	  }
	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
     //hcan->ErrorCode  ���ɻ�ȡ��Ӧ��ESR�Ĵ�����ֵ   ��������ͱ�ʾһ�����������ж��ˣ�������BUSOFF�ж�
}

