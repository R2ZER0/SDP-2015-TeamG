#include <Wire.h>
#include <SDPArduino.h>
#include <PID_v1.h>

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define SAMPLE_TIME 1000

#define NUM_MOTORS 4
#define MOTOR_MOTOR1 2
#define MOTOR_MOTOR2 3
#define MOTOR_MOTOR3 0
#define MOTOR_MOTOR4 5

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

// Set up PID controllers for each wheel
PID* wheel_pids[NUM_MOTORS] = { 0 };

/* Motor movement sensors stuff */
unsigned long next_pid_time = 0L;

const int sample_pid_ms = 100;
const int sample_time_ms = 100;

void rotary_update_positions() {
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
            PID *pid = wheel_pids[i];

            if (desired_speeds[i] < 0) {
                pid->SetOutputLimits(-100, -30);
            } else {
                pid->SetOutputLimits(30, 100);
            }

            pid->Compute();
        }

        // Run motors at these values
        runMotor(MOTOR_MOTOR1, motor_powers[0]);
        runMotor(MOTOR_MOTOR2, motor_powers[1]);
        runMotor(MOTOR_MOTOR3, motor_powers[2]);
        runMotor(MOTOR_MOTOR4, motor_powers[3]);
        
        Serial.print("Motor1");
        Serial.print("\tSpeed: ");
        Serial.print(wheel_speeds[0]);
        Serial.print("\tPower: ");
        Serial.print(motor_powers[0]);
        Serial.println();

        next_pid_time = millis() + sample_pid_ms;
        
    }
}

void setup() {
    SDPsetup();
    digitalWrite(8, HIGH);  // Radio on
    Serial.begin(115200);  // Serial at given baudrate
    Wire.begin();  // Master of the I2C bus
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        PID *pid = new PID(&(wheel_speeds[i]), &(motor_powers[i]), &(desired_speeds[i]), 
                                0.5, 0.0, 0.25, DIRECT);
        pid->SetMode(AUTOMATIC);
        pid->SetSampleTime(sample_time_ms);
    }
    
    while(Serial.available() > 0) { Serial.read(); }
    
    Serial.println("Ready! Send a key to continue...");
    
    while(Serial.available() == 0);
    while(Serial.available() > 0) { Serial.read(); }    
}

void loop()
{
    rotary_update_positions();
}
