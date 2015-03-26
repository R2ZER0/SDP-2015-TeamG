/* wheels.cpp - attempting to control the speed of the wheels */
#include <Arduino.h>
#include <Wire.h>
#include "wheels.h"
#include "motor.h"
#include "config.h"

struct wheel_control {
    double target_speed;
    double speed;
    double motor_power;
    int movement;
    double prev_error;
};

/* Motor lookup table */
static const int8_t MOTOR[NUM_MOTORS] = {
    MOTOR_MOTOR1,
    MOTOR_MOTOR2,
    MOTOR_MOTOR3,
    MOTOR_MOTOR4
};

/* A controller instance for each wheel */
static struct wheel_control wheels[NUM_MOTORS] = { {0} };

/* Timeout variable used to know if we need to recalculate the motor powers */
static unsigned long next_update_time = 0L;

/* Reset the controller and set the desired speed */
void wheels_set_target_speeds(double* speeds)
{
    for(int i = 0; i < NUM_MOTORS; ++i) {
        wheels[i].target_speed = speeds[i];
    }
    next_update_time = 0L;
}

/* Get the last known speed of this wheel */
double wheels_get_speed(int wheel) {
    return wheels[wheel].speed;
}

/* Return the sign of a number */
static double signof(double a) {
    if(a < 0) { return -1; }
    else if(a > 0) { return 1; }
    else { return 0; }
}

/* Calculate the next step of the PD controller */
static void wheel_control_calculate(struct wheel_control* wheel)
{
    double error = (wheel->target_speed - wheel->speed);
   
    double d_error = error - wheel->prev_error;
    wheel->prev_error = error;
    
    double delta = (WHEELS_KP * error) + (WHEELS_KD * d_error);
    wheel->motor_power += delta;
    
    /* Enforce limits */
    if(abs(wheel->motor_power) > WHEELS_POWER_MAX) {
        wheel->motor_power = signof(wheel->motor_power) * WHEELS_POWER_MAX;
    } else if(abs(wheel->motor_power) < WHEELS_POWER_MIN)  {
        wheel->motor_power = signof(wheel->motor_power) * WHEELS_POWER_MIN;
    }
}

void setup_wheels() {}
void service_wheels()
{
    /* First update the wheel movements from rotary slave board */
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, NUM_MOTORS);
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        wheels[i].movement -= (int8_t) Wire.read();  // Must cast to signed 8-bit type    
    }
  
    /* If needed, recalculate wheel speeds and update motors */
    if(millis() > next_update_time) {

        for (int i = 0; i < NUM_MOTORS; i++) {
            struct wheel_control* wheel = &wheels[i];
            
            /* Calculate average wheel speed since last update */
            wheel->speed = (float) (wheel->movement) * (1000.0/WHEELS_UPDATE_INTERVAL);
            wheel->movement = 0;
            
            /* Update the controller */
            wheel_control_calculate(wheel);
            
            /* Send the result to the motor */
            runMotor(MOTOR[i], wheel->motor_power);
        }
        
        next_update_time = millis() + WHEELS_UPDATE_INTERVAL;
    }
}