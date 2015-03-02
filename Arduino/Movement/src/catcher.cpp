////////////////////////////////////////////////////////////////////////////////
// Catching
////////////////////////////////////////////////////////////////////////////////
#include "catcher.h"
#include "config.h"
#include <Arduino.h>
#include <SDPArduino.h>
#include "command.h"

char catcher_state = CATCHER_STATE_STOPPED;
unsigned long catcher_stop_time = 0L;

char catcher_get_state(void)
{
    return catcher_state;
}

static void on_new_command(int cmd, int spd)
{
    if(cmd == CATCHER_COMMAND_IDLE) {
        motorStop(MOTOR_CATCHER);
        command_finished_catcher();
        
    } else if(cmd == CATCHER_COMMAND_CATCH) {
        catcher_catch(spd);
        
    } else if(cmd == CATCHER_COMMAND_RELEASE) {
        catcher_release(spd);   
    }
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

void setup_catcher(void)
{
    motorStop(MOTOR_CATCHER);
    command_sethook_catcher(&on_new_command);
}

void service_catcher(void)
{
    /* Check the catcher state */
    if(catcher_state != CATCHER_STATE_STOPPED) {
        if(millis() >= catcher_stop_time) {
            motorStop(MOTOR_CATCHER);
            catcher_state = CATCHER_STATE_STOPPED;
            command_finished_catcher();
        }
    }
}


