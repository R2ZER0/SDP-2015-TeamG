////////////////////////////////////////////////////////////////////////////////
// Motor Control
////////////////////////////////////////////////////////////////////////////////
#include "motor.h"
#include <SDPArduino.h>
#include "config.h"

static int prev_motor_speed[NUM_MOTORS] = { 0 };

void runMotor(int motor, int motor_speed)
{    
    if(motor_speed == prev_motor_speed[motor]) {
        prev_motor_speed[motor] = 0;
        return;
    }
    
    if(motor_speed == 0) {
        motorStop(motor);
        
    } else if(motor_speed > 0) {
        motorForward(motor, motor_speed);
        
    } else if(motor_speed < 0) {
        motorBackward(motor, -motor_speed);
    }
}