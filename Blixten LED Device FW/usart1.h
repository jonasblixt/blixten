#ifndef __USART1_H_
#define __USART1_H_

#include "stm32f30x.h"

#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32

void usart1_init(void);
int usart1_puts(char *ptr, u8 size);
int usart1_read(char *ptr, u8 size);


#endif