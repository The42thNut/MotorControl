#ifndef COM_CHASSIS
#define COM_CHASSIS

#include "main.h"

void com_send_data(uint16_t std_id,uint8_t* numx,int dlc);
void com_relocate(uint16_t distance);
void com_finish_invert(void);
void com_close_finish(void);

#endif
