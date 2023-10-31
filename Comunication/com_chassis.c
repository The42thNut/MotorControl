#include "com_chassis.h"
#include "can_database.h"
#include "can.h"
#include "usart.h"

/*std id*/
/*
上层CAN1接底盘的CAN2

重定位：
	先去第一个点（坡前） ——   修改模式 遥控/路径上坡 —— 上坡后给上层发
	给上层发           0x12+dlc 2 +0x11 0x22
	收到上层重定位信息 0x21+dlc 4             为激光测距距离

取环：
	底盘到取环第一个位置给上层发 0x89 + dlc 2 + 0x12 0x23      到第1个点（到环之前）后发；
	上层翻下来后底盘收           0x98 + dlc 2 + 0x23 0x34		  收到后去第2个点（去夹住环）；
	底盘到取环第二个位置给上层发 0x89 + dlc 2 + 0x34 0x45      到第2个点后发 ；
	上层夹紧后底盘收到后往回走   0x98 + dlc 2 + 0x45 0x56			收到后去3个点（往前走一点）；

射环：
底盘到达发射点后给上层发消息，视觉信息  0x45+dlc         到达发射点 发；

*/

void com_send_data(uint16_t std_id,uint8_t* numx,int dlc){
	CAN_TxHeaderTypeDef TxMessage;
	int sendnum=0;//发送次数，防止阻滞
	
	TxMessage.StdId=std_id;
	TxMessage.ExtId=0;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;//数据帧
	TxMessage.DLC=dlc;
	TxMessage.TransmitGlobalTime=ENABLE;//时间戳
	
	uint32_t pTxMailbox=0;//返回值，用的哪个邮箱做发送
	HAL_CAN_AddTxMessage(&hcan2, &TxMessage, numx,  &pTxMailbox);
}

void com_relocate(uint16_t distance){//给底盘发送重定位距离信息
	uint8_t numx[4]={0};
	numx[0]=distance>>24;
	numx[1]=distance>>16;
	numx[2]=distance>>8;
	numx[3]=distance;
	com_send_data(0x21,numx,4);
}

void com_finish_invert(void){//上层取环翻转下来后给底盘发送消息
	static int i=0;
	if(i==0){//只发一次
		uint8_t numx[2]={0};
		numx[0]=0x23;
		numx[1]=0x34;
		com_send_data(0x98,numx,2);
		i=1;
	}
}

void com_close_finish(void){//上层将环夹紧后给底盘发送消息
	static int i=0;
	if(i==0){
		uint8_t numx[2]={0};
		numx[0]=0x45;
		numx[1]=0x56;
		com_send_data(0x98,numx,2);
		i=1;
	}
}
