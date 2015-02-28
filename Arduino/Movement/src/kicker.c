////////////////////////////////////////////////////////////////////////////////
// Kicking
////////////////////////////////////////////////////////////////////////////////
#include "kicker.h"

static char kicker_state = KICKER_STATE_STOPPED;
static unsigned long kicker_stop_time = 0L;
static int kicker_scale = KICKER_DEFAULT_SCALE;

char kicker_get_state(void) {
    return kicker_state;
}

void kicker_kick(int kicker_scale)
{
    if(kicker_scale <= 40 || kicker_scale > 100) {
        kicker_scale = KICKER_DEFAULT_SCALE;
    }
  
    kicker_stop_time = millis() + (KICKER_RUNNING_TIME * (100/kicker_scale));
    
    motorForward(MOTOR_KICKER, kicker_scale);
    kicker_state = KICKER_STATE_KICKING;
}

static void kicker_return(void)
{
    kicker_stop_time = millis() + (KICKER_RETURNING_TIME * (100/kicker_scale));
    
    motorBackward(MOTOR_KICKER, kicker_scale);
    kicker_state = KICKER_STATE_RETURNING;    
}

void service_kicker(void) {
    /* Check the kicker state */
    if(kicker_state != KICKER_STATE_STOPPED) {
        if(millis() >= kicker_stop_time) {
            
            if(kicker_state == KICKER_STATE_KICKING) {
                /* When the kicker has finished kicking, it needs to return */
                kicker_return();            
                    
            } else if(kicker_state == KICKER_STATE_RETURNING) {
                /* Finally when finished returning, go to rest */
                motorStop(MOTOR_KICKER);
                kicker_state = KICKER_STATE_STOPPED;
            }
            
        }
    }
}