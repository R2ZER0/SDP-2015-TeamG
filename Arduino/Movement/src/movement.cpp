////////////////////////////////////////////////////////////////////////////////
// Movement
////////////////////////////////////////////////////////////////////////////////
#include "movement.h"
#include "Angle.h"
#include <Arduino.h>
#include <SDPArduino.h>
#include "config.h"
#include "command.h"
#include "math.h"
#include "motor.h"
#include "MPU.h"

int current_command = MOVEMENT_COMMAND_STOP;

float targetAngle = 0.0; // in radians, +PI to -PI
int turnSpeed = 100;
bool finishedTurn = true;
float lastDistance = PI;

float normalise_angle(float a) {
    while(a > PI) { a -= PI; }
    while(a < (0-PI)) { a += PI; }
    return a;
}

float angular_distance(float x, float y) {
    return atan2(sin(x-y), cos(x-y));
}

void calc_motor_speeds(float angle, int scale, int* speed) {
    static const float motor_angle[NUM_MOTORS] = {PI/4.0, PI*3.0/4.0, PI*5.0/4.0, PI*7.0/4.0};
    
    for(int i = 0; i < NUM_MOTORS; ++i) {
        speed[i] = (int)( (float)scale * (cos(angle) * cos(motor_angle[i]) - sin(angle) * sin(motor_angle[i])) );
    }
}

void on_new_command(int cmd, float dir, int spd)
{
    if(cmd == MOVEMENT_COMMAND_STOP) {
        current_command = cmd;
        motorAllStop();
        command_finished_movement();
        
    } else if(cmd == MOVEMENT_COMMAND_MOVE) {
        current_command = cmd;
        
        // Set motors
        int speeds[4];
        calc_motor_speeds(dir, spd, (int*)&speeds);
        runMotor(MOTOR_MOTOR1, speeds[0]);
        runMotor(MOTOR_MOTOR2, speeds[1]);
        runMotor(MOTOR_MOTOR3, speeds[2]);
        runMotor(MOTOR_MOTOR4, speeds[3]);
        
        command_finished_movement();        
        
    } else if(cmd == MOVEMENT_COMMAND_TURN) {
        current_command = cmd;
        
        targetAngle = normalise_angle(dir);
        turnSpeed = spd;
        finishedTurn = false;
        lastDistance = angular_distance(getAngle(), targetAngle);
        
        // Now, we wait...
    }
}

void setup_movement()
{
    motorAllStop();
    command_sethook_movement(&on_new_command);
}

// Very useful: http://stackoverflow.com/questions/7242546/move-from-angle-a-to-b-find-shortest-direction
void service_movement()
{
    if(!finishedTurn && current_command == MOVEMENT_COMMAND_TURN) {
        float current_distance = angular_distance(getAngle(), targetAngle);
        if((current_distance > lastDistance) && (current_distance <= TURN_ACCEPTABLE_RANGE)) {
            // We've probably finished!
            motorAllStop();
            finishedTurn = true;
            command_finished_movement();
            return;
        }
        
        float acw_distance = abs(PI - getAngle()) + abs(-PI - targetAngle);
        float  cw_distance = targetAngle - getAngle();
        
        if(acw_distance < cw_distance) {
            runMotor(MOTOR_MOTOR1, turnSpeed);
            runMotor(MOTOR_MOTOR2, turnSpeed);
            runMotor(MOTOR_MOTOR3, turnSpeed);
            runMotor(MOTOR_MOTOR4, turnSpeed);            
        } else {
            runMotor(MOTOR_MOTOR1, -turnSpeed);
            runMotor(MOTOR_MOTOR2, -turnSpeed);
            runMotor(MOTOR_MOTOR3, -turnSpeed);
            runMotor(MOTOR_MOTOR4, -turnSpeed);
        }
    }
}