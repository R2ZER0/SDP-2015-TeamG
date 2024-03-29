#include <Wire.h>
#include <SDPArduino.h>

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define SAMPLE_TIME 1000

#define NUM_MOTORS 1
#define MOTOR_MOTOR1 2
#define MOTOR_MOTOR2 3
#define MOTOR_MOTOR3 0
#define MOTOR_MOTOR4 5

const int MOTOR[] = { MOTOR_MOTOR1, MOTOR_MOTOR2, MOTOR_MOTOR3, MOTOR_MOTOR4 };

#define KP 1.0
#define KI 0.0
#define KD 0.5

#define SETPOINT 80.0

void runMotor(int motor, int motor_speed)
{
    if(motor_speed == 0) {
        motorStop(motor);
        
    } else if(motor_speed > 0) {
        motorForward(motor, motor_speed);
        
    } else if(motor_speed < 0) {
        motorBackward(motor, -motor_speed);
    }
}

// The desired speeds for motors are set here, and the PID controller attempts
// to match these on each loop
double desired_speeds[NUM_MOTORS] = { 0 };

// Tracks the wheel encoder values
int32_t wheel_movement[NUM_MOTORS] = { 0 };

// The calculated wheel speeds, in encodings/second
double wheel_speeds[NUM_MOTORS] = { 0 };

// PID Calculated output for motor powers
double motor_powers[NUM_MOTORS] = { 0 };

double wheel_prev_error[NUM_MOTORS] = { 0.0 };

/* Motor movement sensors stuff */
unsigned long next_pid_time = 0L;

const int sample_time_ms = 200;

void * operator new (size_t size, void * ptr) { return ptr; }

void setup() {
    SDPsetup();
    Serial.println("STARTING");
    
    motorAllStop();
    
    for (int i = 0; i < NUM_MOTORS; i++) {        
        desired_speeds[i] = SETPOINT;
    }
}

double signof(double a) {
    if(a < 0) { return -1; }
    else if(a > 0) { return 1; }
    else { return 0; }
}

void loop() {
    
    // Request motor position deltas from rotary slave board
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, NUM_MOTORS);
    
    // Update the recorded motor positions
    for (int i = 0; i < NUM_MOTORS; i++) {
        wheel_movement[i] -= (int8_t) Wire.read();  // Must cast to signed 8-bit type    
    }
  
    if(millis() > next_pid_time) {

        for (int i = 0; i < NUM_MOTORS; i++) {
            wheel_speeds[i] = (double) (wheel_movement[i]) * (1000.0/(double)sample_time_ms);
            wheel_movement[i] = 0;
        }
        
        /* Computes new PID expected values for desired speeds and update motors. */
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
        for(int i = 0; i < NUM_MOTORS; ++i) {
            runMotor(MOTOR[i], (int)motor_powers[i]);
        }
        
        Serial.print("\tTarget: ");
        Serial.print(desired_speeds[0]);
        Serial.print("\tSpeed: ");
        Serial.print(wheel_speeds[0]);
        Serial.print("\tPower: ");
        Serial.print(motor_powers[0]);
        Serial.println();
        
        next_pid_time = millis() + sample_time_ms;
        
    }
}