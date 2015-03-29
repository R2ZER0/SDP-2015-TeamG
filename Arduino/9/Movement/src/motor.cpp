////////////////////////////////////////////////////////////////////////////////
// Motor Control
////////////////////////////////////////////////////////////////////////////////
#include "motor.h"
#include <SDPArduino.h>
#include "config.h"

static int prev_motor_speed[NUM_MOTORS] = { 0 };

void runMotor(int motor, int motor_speed)
{    
    if(motor_speed == prev_motor_speed[motor]) { return; }
    
    prev_motor_speed[motor] = motor_speed;
    
    if(motor_speed == 0) {
        motorStop(motor);
        
    } else if(motor_speed > 0) {
        motorForward(motor, motor_speed);
        
    } else if(motor_speed < 0) {
        motorBackward(motor, -motor_speed);
    }
}