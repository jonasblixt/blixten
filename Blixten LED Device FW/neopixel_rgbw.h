#ifndef __NEOPIXEL_RGBW_H
#define __NEOPIXEL_RGBW_H


#include "stm32f30x.h"

void neopixel_rgbw_init(void);
int neopixel_rgbw_set_led(u8 led_index, u32 rgbw_value);
u16 neopixel_get_framecount(void);
void neopixel_update_buffer(void);
u8 neopixel_buffer_available(void);

#endif