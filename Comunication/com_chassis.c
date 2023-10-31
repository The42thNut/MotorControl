#include "com_chassis.h"
#include "can_database.h"
#include "can.h"
#include "usart.h"

/*std id*/
/*
�ϲ�CAN1�ӵ��̵�CAN2

�ض�λ��
	��ȥ��һ���㣨��ǰ�� ����   �޸�ģʽ ң��/·������ ���� ���º���ϲ㷢
	���ϲ㷢           0x12+dlc 2 +0x11 0x22
	�յ��ϲ��ض�λ��Ϣ 0x21+dlc 4             Ϊ���������

ȡ����
	���̵�ȡ����һ��λ�ø��ϲ㷢 0x89 + dlc 2 + 0x12 0x23      ����1���㣨����֮ǰ���󷢣�
	�ϲ㷭�����������           0x98 + dlc 2 + 0x23 0x34		  �յ���ȥ��2���㣨ȥ��ס������
	���̵�ȡ���ڶ���λ�ø��ϲ㷢 0x89 + dlc 2 + 0x34 0x45      ����2����� ��
	�ϲ�н�������յ���������   0x98 + dlc 2 + 0x45 0x56			�յ���ȥ3���㣨��ǰ��һ�㣩��

�价��
���̵��﷢������ϲ㷢��Ϣ���Ӿ���Ϣ  0x45+dlc         ���﷢��� ����

*/

void com_send_data(uint16_t std_id,uint8_t* numx,int dlc){
	CAN_TxHeaderTypeDef TxMessage;
	int sendnum=0;//���ʹ�������ֹ����
	
	TxMessage.StdId=std_id;
	TxMessage.ExtId=0;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;//����֡
	TxMessage.DLC=dlc;
	TxMessage.TransmitGlobalTime=ENABLE;//ʱ���
	
	uint32_t pTxMailbox=0;//����ֵ���õ��ĸ�����������
	HAL_CAN_AddTxMessage(&hcan2, &TxMessage, numx,  &pTxMailbox);
}

void com_relocate(uint16_t distance){//�����̷����ض�λ������Ϣ
	uint8_t numx[4]={0};
	numx[0]=distance>>24;
	numx[1]=distance>>16;
	numx[2]=distance>>8;
	numx[3]=distance;
	com_send_data(0x21,numx,4);
}

void com_finish_invert(void){//�ϲ�ȡ����ת����������̷�����Ϣ
	static int i=0;
	if(i==0){//ֻ��һ��
		uint8_t numx[2]={0};
		numx[0]=0x23;
		numx[1]=0x34;
		com_send_data(0x98,numx,2);
		i=1;
	}
}

void com_close_finish(void){//�ϲ㽫���н�������̷�����Ϣ
	static int i=0;
	if(i==0){
		uint8_t numx[2]={0};
		numx[0]=0x45;
		numx[1]=0x56;
		com_send_data(0x98,numx,2);
		i=1;
	}
}
