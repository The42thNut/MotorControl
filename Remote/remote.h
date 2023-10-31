#ifndef REMOTE_UART_H
#define REMOTE_UART_H

#include "usart.h"

typedef __packed struct
{
  
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  uint8_t sw1;
  uint8_t sw2;
	int16_t cir;
} rc_info_t;

extern rc_info_t rc;
extern volatile int waittime;

void Rc_callback_handler(rc_info_t *rc, uint8_t *buff);
#endif
