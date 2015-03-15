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
        //lastDistance = angular_distance(getAngle(), targetAngle);
        
        // Now, we wait...
    }
}

void setup_movement()
{
    motorAllStop();
}

void rotary_update_positions();

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
        
        if(current_distance < 0.2) {
            turnSpeedA = 60;
            turnSpeedB = -40;
        
        } else if(current_distance < 1.5) {
            turnSpeedB = 50;
            turnSpeedA = 0;
        } else {
            turnSpeedA = 0;
            turnSpeedB = 60;
        }
        
        if(acw_dist < cw_dist) {
            runMotor(MOTOR_MOTOR1, turnSpeedA);
            runMotor(MOTOR_MOTOR2, turnSpeedB);
            runMotor(MOTOR_MOTOR3, turnSpeedA);
            runMotor(MOTOR_MOTOR4, turnSpeedB);            
        } else {
            runMotor(MOTOR_MOTOR1, 0-turnSpeedA);
            runMotor(MOTOR_MOTOR2, 0-turnSpeedB);
            runMotor(MOTOR_MOTOR3, 0-turnSpeedA);
            runMotor(MOTOR_MOTOR4, 0-turnSpeedB);
        }
        
        //Serial.print("dist="); Serial.println(current_distance);
    }
    
    rotary_update_positions();
}


/* Motor movement sensors stuff */
int32_t wheel_movement[ROTARY_COUNT] = { 0 };

void rotary_update_positions() {
  // Request motor position deltas from rotary slave board
  Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
  
  // Update the recorded motor positions
  for (int i = 0; i < ROTARY_COUNT; i++) {
    wheel_movement[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
  }
  
  Serial.print("Wheels: ");
  Serial.print(wheel_movement[0]); Serial.print(' ');
  Serial.print(wheel_movement[1]); Serial.print(' ');
  Serial.print(wheel_movement[2]); Serial.print(' ');
  Serial.println(wheel_movement[3]);
}
