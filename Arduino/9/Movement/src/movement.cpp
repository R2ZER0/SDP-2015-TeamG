////////////////////////////////////////////////////////////////////////////////
// Movement
////////////////////////////////////////////////////////////////////////////////
#include "movement.h"
#include <Arduino.h>
#include <SDPArduino.h>
#include "config.h"
#include "command.h"
#include "math.h"
#include "motor.h"
#include "MPU.h"
#include <Wire.h>
#include "wheels.h"

char current_command = MOVEMENT_COMMAND_STOP;

float targetAngle = 0.0; // in radians, +PI to -PI
int turnSpeed = 100;
bool finishedTurn = true;

float normalise_angle(float a) {
    while(a > PI) { a -= 2.0*PI; }
    while(a <= (-PI)) { a += 2.0*PI; }
    return a;
}

void calc_motor_speeds(float angle, int scale, double* speed) {
    static const float motor_angle[NUM_MOTORS] = {PI/4.0, PI*3.0/4.0, PI*5.0/4.0, PI*7.0/4.0};
    
    for(int i = 0; i < NUM_MOTORS; ++i) {
        speed[i] = (double)( (float)scale * (cos(angle) * cos(motor_angle[i]) - sin(angle) * sin(motor_angle[i])) );
    }
}

void movement_on_new_command(char cmd, float dir, int spd)
{
    if(cmd == MOVEMENT_COMMAND_STOP) {

        wheels_stop();
        wheels_control_enabled(false);

        current_command = cmd;
        motorStop(MOTOR_MOTOR1);
        motorStop(MOTOR_MOTOR2);
        motorStop(MOTOR_MOTOR3);
        motorStop(MOTOR_MOTOR4);
        command_finished_movement();
        
    } else if(cmd == MOVEMENT_COMMAND_MOVE) {
        current_command = cmd;
        
        // Set motors
        double speeds[NUM_MOTORS];
        calc_motor_speeds(dir, spd, speeds);

        // Set desired motor speeds
        wheels_control_enabled(true);
        wheels_set_target_speeds(speeds);

        // We are done! (?)
        command_finished_movement();        
        
    } else if(cmd == MOVEMENT_COMMAND_TURN) {
        current_command = cmd;
        
        wheels_control_enabled(false);
        targetAngle = normalise_angle(getAngle() + dir);
        turnSpeed = spd;
        finishedTurn = false;
    }
}

void setup_movement()
{
    motorAllStop();
}

float acw_distance(float a, float b) { return (b-a)<0 ? (b - a + PI*2.0) : (b - a); }
float  cw_distance(float a, float b) { return (a-b)<0 ? (a - b + PI*2.0) : (a - b); }

void service_movement()
{
    if(!finishedTurn && current_command == MOVEMENT_COMMAND_TURN) {
        float acw_dist = acw_distance(getAngle(), targetAngle);
        float cw_dist = cw_distance(getAngle(), targetAngle);
        float current_distance = (acw_dist < cw_dist) ? acw_dist : cw_dist;
        
        // Serial.print(getAngle()); Serial.print(" --> "); Serial.print(targetAngle);
        // Serial.print(" = "); Serial.println(current_distance);
        //
        
        if(current_distance <= TURN_ACCEPTABLE_RANGE) {
            // We've probably finished!
            motorAllStop();
            finishedTurn = true;
            command_finished_movement();
            Serial.println("Finished turn!");
            return;
        }
        
        // Slow down as we approach the target
        int turnSpeedA = turnSpeed;
        int turnSpeedB = turnSpeed;
        if(current_distance < 0.5) {
            turnSpeedA = 40;
            turnSpeedB = 0;
        } else if(current_distance < 1.5) {
            turnSpeedA = 40;
            turnSpeedB = 0;
        } else {
            turnSpeedA = 50;
            turnSpeedB = 0;
        }
        
        //Serial.print(current_distance);
        //Serial.print('\t');
        //Serial.println(turnSpeedA);
        
        if(acw_dist < cw_dist) {
            runMotor(MOTOR_MOTOR1, turnSpeedA);
            runMotor(MOTOR_MOTOR2, turnSpeedB);
            runMotor(MOTOR_MOTOR3, turnSpeedA);
            runMotor(MOTOR_MOTOR4, turnSpeedB);
        } else {
            runMotor(MOTOR_MOTOR1, -turnSpeedA);
            runMotor(MOTOR_MOTOR2, -turnSpeedB);
            runMotor(MOTOR_MOTOR3, -turnSpeedA);
            runMotor(MOTOR_MOTOR4, -turnSpeedB);
        }
        
        //Serial.print("dist="); Serial.println(current_distance);
    }
}
