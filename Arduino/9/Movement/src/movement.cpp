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
#include <Wire.h>
#include "wheels.h"

char current_command = MOVEMENT_COMMAND_STOP;

float targetAngle = 0.0; // in radians, +PI to -PI
int turnSpeed = 100;
bool finishedTurn = true;
float lastDistance = PI;

float normalise_angle(float a) {
    while(a > PI) { a -= 2.0*PI; }
    while(a <= (-PI)) { a += 2.0*PI; }
    return a;
}

void calc_motor_speeds(float angle, int scale, int* speed) {
    static const float motor_angle[NUM_MOTORS] = {PI/4.0, PI*3.0/4.0, PI*5.0/4.0, PI*7.0/4.0};
    
    for(int i = 0; i < NUM_MOTORS; ++i) {
        speed[i] = (int)( (float)scale * (cos(angle) * cos(motor_angle[i]) - sin(angle) * sin(motor_angle[i])) );
    }
}

void movement_on_new_command(char cmd, float dir, int spd)
{
    if(cmd == MOVEMENT_COMMAND_STOP) {

        // Reset desired speeds down to 0
        for (int i = 0; i < NUM_MOTORS; i++) {
            desired_speeds[i] = 0;
        }

        current_command = cmd;
        motorStop(MOTOR_MOTOR1);
        motorStop(MOTOR_MOTOR2);
        motorStop(MOTOR_MOTOR3);
        motorStop(MOTOR_MOTOR4);
        command_finished_movement();
        
    } else if(cmd == MOVEMENT_COMMAND_MOVE) {
        current_command = cmd;
        
        // Set motors
        int speeds[4];
        calc_motor_speeds(dir, spd, (int*)&speeds);

        // Set desired motor speeds
        for (int i = 0; i < NUM_MOTORS; i++) {
            wheels_set_target_speed(i, (double) speeds[i]);
        }

        command_finished_movement();        
        
    } else if(cmd == MOVEMENT_COMMAND_TURN) {
        current_command = cmd;
        
        targetAngle = normalise_angle(dir);
        turnSpeed = spd;
        finishedTurn = false;
        //lastDistance = angular_distance(getAngle(), targetAngle);
        
        // Now, we wait...
    }
}

void setup_movement()
{
    motorAllStop();
}

void rotary_update_positions();
void compute_pid();

//float acw_distance(float a, float b) { return  }
//float  cw_distance(float a, float b) { return abs(normalise_angle(b - a)); }

//float acw_distance(float x, float y) { return normalise_angle( atan2(cos(y), sin(y)) - atan2(cos(x), sin(y)) ); }
//float acw_distance(float x, float y) { return PI - abs(abs(x - y) - PI); }
//float  cw_distance(float x, float y) { return 2.0*PI - acw_distance(x, y); }

float acw_distance(float a, float b) { return (b-a)<0 ? (b - a + PI*2.0) : (b - a); }
float  cw_distance(float a, float b) { return (a-b)<0 ? (a - b + PI*2.0) : (a - b); }

void service_movement()
{
    if(!finishedTurn && current_command == MOVEMENT_COMMAND_TURN) {
        float acw_dist = acw_distance(getAngle(), targetAngle);
        float  cw_dist =  cw_distance(getAngle(), targetAngle);
        float current_distance = (acw_dist < cw_dist) ? acw_dist : cw_dist;
        
//         Serial.print(getAngle()); Serial.print(" --> "); Serial.print(targetAngle);
//         Serial.print(" = "); Serial.println(current_distance);
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
        
        Serial.print(current_distance);
        Serial.print('\t');
        Serial.println(turnSpeedA);
        
        if(acw_dist > cw_dist) {
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

double signof(double a) {
    if(a < 0) { return -1; }
    else if(a > 0) { return 1; }
    else { return 0; }
}
