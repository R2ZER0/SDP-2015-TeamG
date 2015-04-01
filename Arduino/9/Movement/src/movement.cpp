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

double targetAngle = 0.0; // in radians, +PI to -PI
int turnSpeed = 100;
bool finishedTurn = true;
static unsigned long turn_quickstart_endtime = 0L;

double normalise_angle(double a) {
    while(a > PI) { a -= 2.0*PI; }
    while(a <= (-PI)) { a += 2.0*PI; }
    return a;
}

void calc_motor_speeds(double angle, int scale, double* speed) {
    static const double motor_angle[NUM_MOTORS] = {PI/4.0, PI*3.0/4.0, PI*5.0/4.0, PI*7.0/4.0};
    
    for(int i = 0; i < NUM_MOTORS; ++i) {
        speed[i] = (double)( (double)scale * (cos(angle) * cos(motor_angle[i]) - sin(angle) * sin(motor_angle[i])) );
    }
}

void movement_on_new_command(char cmd, float dir_, int spd)
{
    double dir = (double) dir_;
    
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
        
        wheels_control_enabled(true);
        targetAngle = normalise_angle(getAngle() + dir);
        turnSpeed = spd;
        finishedTurn = false;
        
        turn_quickstart_endtime = millis() + TURN_QUICKSTART_TIME;
    }
}

void setup_movement()
{
    motorAllStop();
}

double acw_distance(double a, double b) { return (b-a)<0 ? (b - a + PI*2.0) : (b - a); }
double  cw_distance(double a, double b) { return (a-b)<0 ? (a - b + PI*2.0) : (a - b); }

void service_movement()
{
    if(!finishedTurn && current_command == MOVEMENT_COMMAND_TURN) {
        double acw_dist = acw_distance(getAngle(), targetAngle);
        double cw_dist = cw_distance(getAngle(), targetAngle);
        double current_distance = (acw_dist < cw_dist) ? acw_dist : cw_dist;
        
        // Serial.print(getAngle()); Serial.print(" --> "); Serial.print(targetAngle);
        // Serial.print(" = "); Serial.println(current_distance);
        //
        
        if(current_distance <= TURN_ACCEPTABLE_RANGE) {
            // We've probably finished!
            
            wheels_stop();
            
            runMotor(MOTOR_MOTOR1, 0);
            runMotor(MOTOR_MOTOR2, 0);
            runMotor(MOTOR_MOTOR3, 0);
            runMotor(MOTOR_MOTOR4, 0);
            
            finishedTurn = true;
            command_finished_movement();
            //Serial.println("Finished turn!");
            return;
        }
        
        // Slow down as we approach the target
//         if(current_distance < 0.5) {
//             turnSpeed = 30;
//         } else if(current_distance < 1.5) {
//             turnSpeed = 30;
//         } else {
//             turnSpeed = 40;
//         }
        turnSpeed = 25;
        
        //Serial.print(current_distance);
        //Serial.print('\t');
        //Serial.println(turnSpeedA);
        
        double speeds[NUM_MOTORS];
        
        if(acw_dist < cw_dist) {
            speeds[0] = turnSpeed;
            speeds[1] = turnSpeed;
            speeds[2] = turnSpeed;
            speeds[3] = turnSpeed;
        } else {
            speeds[0] = -turnSpeed;
            speeds[1] = -turnSpeed;
            speeds[2] = -turnSpeed;
            speeds[3] = -turnSpeed;
        }
        
        wheels_set_target_speeds(speeds);
        
        //Serial.print("dist="); Serial.println(current_distance);
    }
}
