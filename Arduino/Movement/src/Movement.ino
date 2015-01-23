#include <Wire.h>
#include <SDPArduino.h>
#include <SerialCommand.h>

const int PIN_LED   = 13;

SerialCommand comm;

void setup()
{  
  // SDP Setup
  SDPsetup();
  
  // Setup LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  // Initialse the command messenger
  comm.addCommand("RUN",   cmd_RUN);
  comm.addCommand("PING",  cmd_PING);
  comm.addCommand("LED",   cmd_LED);
  
  Serial.println("STARTED");
}

int led_state = HIGH;

void cmd_LED()
{
  char* arg = comm.next();
  if(arg == NULL) {
    // Flip the state if no argument given
    switch(led_state) {
      case HIGH: led_state = LOW;
      case  LOW: led_state = HIGH; 
    }
    
  } else if(arg == "HIGH") {
    led_state = HIGH;
    
  } else if(arg == "LOW") {
    led_state = LOW;
  }
  
  digitalWrite(PIN_LED, HIGH);
}

void cmd_PING()
{
  Serial.println("PONG");
}

void cmd_RUN()
{
  int motor1 = 0.0;
  int motor2 = 0.0;
  int motor3 = 0.0;
  char* arg;
  
  arg = comm.next();
  if(arg != NULL) {
    motor1 = atoi(arg);
  }
  
  arg = comm.next();
  if(arg != NULL) {
    motor2 = atoi(arg);
  }
  
  arg = comm.next();
  if(arg != NULL) {
    motor3 = atoi(arg);
  }
  
  doRun(motor1, motor2, motor3);
}

void loop()
{
  comm.readSerial();
}

void doRun(int motor1, int motor2, int motor3)
{
  doRunMotor(0, motor1);
  doRunMotor(1, 0 - motor2); // Motor 2 is backwards
  doRunMotor(2, 0 - motor3); // Motor 3 is backwards
}

void doRunMotor(int motor, int motor_speed)
{
  if(motor_speed == 0) {
    motorStop(motor);
    
  } else if(motor_speed > 0) {
    motorForward(motor, motor_speed);
    
  } else if(motor_speed < 0) {
    motorBackward(motor, -motor_speed);
  }
}


