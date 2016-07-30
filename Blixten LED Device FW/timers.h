#ifndef TIMERS_H_
#define TIMERS_H_

#include "stm32f30x.h"
#include <stddef.h>

typedef struct {
    volatile u32 timer_expire_value;
    u8 timer_active;
    u32 timer_ticks;
    void (*timer_callback)(void* p);
    void* timer_param;
} t_timer;

void timer_init(t_timer *new_timer, u32 value, void (*callback)(void *p), void* param);
u8 timer_has_expired(t_timer* t);
u8 timer_check_and_reset(t_timer* t);
void timer_reset(t_timer* t);
void timer_task(t_timer* t);
void timer_stop (t_timer *t);
void timer_start (t_timer *t);
u8 timer_set_new_value(t_timer* t, u32 new_value);

#endif /*TIMERS_H_*/

