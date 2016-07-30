
#include "timers.h"

extern unsigned int system_get_tick();

void timer_init(t_timer *new_timer, u32 value, void (*callback)(void *p), void* param)
{
 
    new_timer->timer_expire_value = value;
    new_timer->timer_callback = callback;
    new_timer->timer_param = param;
    new_timer->timer_active = false;
    new_timer->timer_ticks = 0;

}


u8 timer_set_new_value(t_timer* t, u32 new_value)
{
    if (t == NULL)
        return false;
    
    t->timer_expire_value = new_value;
    timer_reset(t);
    
    return true;
}

u8 timer_has_expired(t_timer* t)
{
    if (t == NULL)
        return false;

    if ( (system_get_tick() - t->timer_ticks) > t->timer_expire_value )
        return true;
    else
        return false;
}

u8 timer_check_and_reset(t_timer* t)
{
    if (t == NULL)
        return false;
    u8 has_expired = timer_has_expired(t);
    
    if (has_expired)
        timer_reset(t);
        
    return has_expired;
}

void timer_reset(t_timer* t)
{
    if (t == NULL)
        return;
    t->timer_active = true;
    t->timer_ticks = system_get_tick();
}

void timer_start (t_timer *t)
{
    if (t == NULL)
        return;
    t->timer_active = true;
    t->timer_ticks = system_get_tick();
}

void timer_stop (t_timer *t)
{
    if (t == NULL)
        return;
    t->timer_active = false;
}


void timer_task(t_timer* t)
{    
    if (t == NULL) {
        return;
    }

    if (!(t->timer_active)) {
        return;
    }
       
    if (timer_has_expired(t) && (t->timer_callback != NULL)) {
        timer_reset(t);        
        t->timer_callback( t->timer_param );
    }        
}

