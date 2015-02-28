////////////////////////////////////////////////////////////////////////////////
// Catching
////////////////////////////////////////////////////////////////////////////////
#include "catcher.h"
#include "config.h"
#include <Arduino.h>
#include <SDPArduino.h>

char catcher_state = CATCHER_STATE_STOPPED;
unsigned long catcher_stop_time = 0L;

char catcher_get_state(void) {
    return catcher_state;
}

void catcher_catch(int scale)
{
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
  
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorForward(MOTOR_CATCHER, 100);
    catcher_state = CATCHER_STATE_CATCHING;
}

void catcher_release(int scale)
{
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
    
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorBackward(MOTOR_CATCHER, scale);
    catcher_state = CATCHER_STATE_RELEASING;
}

void service_catcher(void)
{
    /* Check the catcher state */
    if(catcher_state != CATCHER_STATE_STOPPED) {
        if(millis() >= catcher_stop_time) {
            motorStop(MOTOR_KICKER);
            catcher_state = CATCHER_STATE_STOPPED;
        }
    }
}


