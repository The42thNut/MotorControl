#ifndef CAN_DATABASE_H
#define CAN_DATABASE_H

#include "stm32f4xx.h"

#ifndef NULL
	#define NULL ((void *)0)//此处的NULL指示candatabase中的NULL
#endif
#define READ_ONLY  0    //主控读，外设写
#define WRITE_ONLY 1    //主控写，外设读

typedef enum{
	odrive_motor1=0x01,//新增odrive电机号
	
} ID_NUMDEF;

typedef struct
{
    uint8_t  Data_type;
    ID_NUMDEF  Data_ID;
    uint8_t* Data_ptr;
    uint8_t  Data_length;   
    void (*MenuFunc)(void);//入口函数        
    uint8_t  Channel;
    uint8_t  Fifo_num;//在接收方将该ID配置的fifo号
} Can_Data;

/**************Public_begin**************/
extern uint8_t can_data_num_g;;//结构体数组can_database_g的大小，在Hash_table_init(void)中更新
extern Can_Data can_database_g[];
extern uint16_t hash_table[1000];;
void Hash_table_init(void);//初始化通信表格
void Can_start_work(void);//CAN开始工作
/**************Public_end**************/


#endif
