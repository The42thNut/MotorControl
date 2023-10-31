#include "string.h"
#include "stdlib.h"
#include "remote.h"
#include "usart.h"
#include "main.h"

rc_info_t rc;

volatile int remote_failed_time;
volatile int waittime=0;

void Rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
	rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;

  rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (buff[5] >> 4) & 0x0003;
	rc->cir = ((((uint16_t)buff[17]& 0x0007 )<<8) | ( (int16_t)buff[16] & 0x00FF ))-1024;
  
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660)||
				rc->sw1>3||
				rc->sw2>3||
				rc->sw1<0||
				rc->sw2<0)
  {
    memset(rc, 0, sizeof(rc_info_t));
		waittime++;
		remote_failed_time=HAL_GetTick();
  }
	else
	{
		waittime=0;
	}
}

