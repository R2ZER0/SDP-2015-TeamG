#include <Wire.h>
#include <SDPArduino.h>

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6
#define SAMPLE_TIME 1000

/* Motor movement sensors stuff */
int32_t wheel_movement[ROTARY_COUNT] = { 0 };

unsigned long next_print_time = 0L;

void rotary_update_positions() {
    // Request motor position deltas from rotary slave board
    Wire.requestFrom(ROTARY_SLAVE_ADDRESS, ROTARY_COUNT);
    
    // Update the recorded motor positions
    for (int i = 0; i < ROTARY_COUNT; i++) {
        wheel_movement[i] += (int8_t) Wire.read();  // Must cast to signed 8-bit type
    }
  
    if(millis() > next_print_time) {
        Serial.print("Wheels: ");
        Serial.print(wheel_movement[0]); Serial.print(' ');
        Serial.print(wheel_movement[1]); Serial.print(' ');
        Serial.print(wheel_movement[2]); Serial.print(' ');
        Serial.println(wheel_movement[3]);
        
        wheel_movement[0] = 0;
        wheel_movement[1] = 0;
        wheel_movement[2] = 0;
        wheel_movement[3] = 0;
        
        next_print_time = millis() + SAMPLE_TIME;
    }
}


#define MOTOR_MOTOR1  1
#define MOTOR_MOTOR2  3
#define MOTOR_MOTOR3  0
#define MOTOR_MOTOR4  2

void setup() {
    SDPsetup();
    digitalWrite(8, HIGH);  // Radio on
    Serial.begin(115200);  // Serial at given baudrate
    Wire.begin();  // Master of the I2C bus
    
    int speed = 60;
    
    motorForward(MOTOR_MOTOR1, speed);
    motorForward(MOTOR_MOTOR2, speed);
    motorForward(MOTOR_MOTOR3, speed);
    motorForward(MOTOR_MOTOR4, speed);
}

void loop()
{
    rotary_update_positions();
}
