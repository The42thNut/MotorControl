#ifndef CAN_DATABASE_H
#define CAN_DATABASE_H

#include "stm32f4xx.h"

#ifndef NULL
	#define NULL ((void *)0)//�˴���NULLָʾcandatabase�е�NULL
#endif
#define READ_ONLY  0    //���ض�������д
#define WRITE_ONLY 1    //����д�������

typedef enum{
	odrive_motor1=0x01,//����odrive�����
	
} ID_NUMDEF;

typedef struct
{
    uint8_t  Data_type;
    ID_NUMDEF  Data_ID;
    uint8_t* Data_ptr;
    uint8_t  Data_length;   
    void (*MenuFunc)(void);//��ں���        
    uint8_t  Channel;
    uint8_t  Fifo_num;//�ڽ��շ�����ID���õ�fifo��
} Can_Data;

/**************Public_begin**************/
extern uint8_t can_data_num_g;;//�ṹ������can_database_g�Ĵ�С����Hash_table_init(void)�и���
extern Can_Data can_database_g[];
extern uint16_t hash_table[1000];;
void Hash_table_init(void);//��ʼ��ͨ�ű��
void Can_start_work(void);//CAN��ʼ����
/**************Public_end**************/


#endif
