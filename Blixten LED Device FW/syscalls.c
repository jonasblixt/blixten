#include <stdio.h>
#include "syscalls.h"
#include "usart1.h"


int _write (int fd, const void *buf, size_t count)
{
	return usart1_puts((char *) buf, count);
}

int _read (int fd, const void *buf, size_t count){
	return count;
}

