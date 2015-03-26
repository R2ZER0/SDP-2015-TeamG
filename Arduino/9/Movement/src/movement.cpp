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

// The desired speeds for motors are set here, and the PID controller attempts
// to match these on each loop
double desired_speeds[NUM_MOTORS] = { 0 };

// Tracks the wheel encoder values
int32_t wheel_movement[NUM_MOTORS] = { 0 };

// The calculated wheel speeds, in encodings/second
double wheel_speeds[NUM_MOTORS] = { 0 };

// PID Calculated output for motor powers
double motor_powers[NUM_MOTORS] = { 0 };

// Remember the previous errors for D component
double wheel_prev_error[NUM_MOTORS] = { 0.0 };

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
            motor_powers[i] = speeds[i];
            desired_speeds[i] = (double) speeds[i];
            wheel_prev_error[i] = 0;
            wheel_movement[i] = 0;
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
    
    rotary_update_positions();
    compute_pid();
}


/* Motor movement sensors stuff */
unsigned long next_print_time = 0L;
unsigned long next_pid_time = 0L;

const int sample_pid_ms = 200;
const int sample_time_ms = 200;

void rotary_update_positions() {
    // Request motor position deltas from rotary slave board
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, NUM_MOTORS);
    
    // Update the recorded motor positions
    for (int i = 0; i < NUM_MOTORS; i++) {
        wheel_movement[i] -= (int8_t) Wire.read();  // Must cast to signed 8-bit type    
    }
  
    if(millis() > next_print_time) {

        for (int i = 0; i < NUM_MOTORS; i++) {
            wheel_speeds[i] = (float) (wheel_movement[i]) * (1000.0/sample_time_ms);
            wheel_movement[i] = 0;
        }
        
        next_print_time = millis() + sample_time_ms;
    }
}

double signof(double a) {
    if(a < 0) { return -1; }
    else if(a > 0) { return 1; }
    else { return 0; }
}

/* Computes new PID expected values for desired speeds and update motors. */
void compute_pid() {

    if (current_command != MOVEMENT_COMMAND_MOVE) {
        return;
    }

    if(millis() > next_pid_time) {

        for (int i = 0; i < NUM_MOTORS; i++) {
            double error = (desired_speeds[i] - wheel_speeds[i]);
           
            double d_error = error - wheel_prev_error[i];
            wheel_prev_error[i] = error;
            
            double delta = KP * error + KD * d_error;
            motor_powers[i] += delta;
            
            if(abs(motor_powers[i]) > 100) { motor_powers[i] = signof(motor_powers[i]) * 100; }
            if(abs(motor_powers[i]) < 30)  { motor_powers[i] = signof(motor_powers[i]) * 30; }
        }

        // Run motors at these values
        runMotor(MOTOR_MOTOR1, motor_powers[0]);
        runMotor(MOTOR_MOTOR2, motor_powers[1]);
        runMotor(MOTOR_MOTOR3, motor_powers[2]);
        runMotor(MOTOR_MOTOR4, motor_powers[3]);

        next_pid_time = millis() + sample_pid_ms;
    }
}
