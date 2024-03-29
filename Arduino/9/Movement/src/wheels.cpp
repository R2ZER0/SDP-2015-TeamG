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
    unsigned long next_update_time;
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

/* Is wheel control enabled? */
static bool control_enabled = false;

/* Enable/disable wheel control */
void wheels_control_enabled(bool enabled) {
    control_enabled = enabled;
}

/* Reset the controller and set the desired speed */
void wheels_set_target_speeds(double* speeds)
{
    for(int i = 0; i < NUM_MOTORS; ++i) {
        if(wheels[i].target_speed != speeds[i]) {
            wheels[i].target_speed = speeds[i];
            wheels[i].next_update_time = 0L; /* Update immidiately */
        }
    }
}

/* Stop the wheels! */
void wheels_stop(void)
{
    for(int i = 0; i < NUM_MOTORS; ++i) {
        wheels[i].target_speed = 0;
        wheels[i].motor_power = 0;
        wheels[i].next_update_time = 0L; /* Update immidiately */
        runMotor(MOTOR[i], 0);
    }
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
    /* Check if the target speed is in the dead zone, i.e. stopped */
    if(abs(wheel->target_speed) <= WHEELS_DEADZONE_SIZE) {
        wheel->motor_power = 0;
        return;
    }
    
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
    unsigned long current_time = millis();
        
    for (int i = 0; i < NUM_MOTORS; i++) {
        struct wheel_control* wheel = &wheels[i];
        
        if(current_time > wheel->next_update_time) {
            
            /* Calculate average wheel speed since last update */
            wheel->speed = (double) (wheel->movement);
            wheel->speed = wheel->speed * (double)1000.0;
            wheel->speed = wheel->speed / (double)WHEELS_UPDATE_INTERVAL;
            wheel->movement = 0;
            
            if(control_enabled) {
                /* Update the controller */
                wheel_control_calculate(wheel);
                
                /* Send the result to the motor */
                runMotor(MOTOR[i], wheel->motor_power);
            }
            
            wheel->next_update_time = current_time + WHEELS_UPDATE_INTERVAL;
        }
    }
}