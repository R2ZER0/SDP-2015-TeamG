#include <Wire.h>
#include <SDPArduino.h>
#include <SerialCommand.h>

const int PIN_LED   = 13;
const int PIN_RADIO = 8;

SerialCommand comm;

void setup()
{
  // SDP Setup
  SDPsetup();
  
  // Enable setting the LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  // Select the radio for serial
  pinMode(PIN_RADIO, OUTPUT);    
  digitalWrite(PIN_RADIO, HIGH);
  
  // Initialise serial-over-RF
  Serial.begin(115200);
  
  // Initialse the command messenger
  comm.addCommand("PING", cmd_PING);
  comm.addCommand("RUN", cmd_RUN);
}

void cmd_PING()
{
  Serial.println("PONG");
  digitalWrite(PIN_LED, LOW);
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
  doRunMotor(1, motor2);
  doRunMotor(3, motor3);
}

void doRunMotor(int motor, int motor_speed)
{
  if(motor_speed == 0) {
    motorStop(motor);
    
  } else if(motor_speed > 0) {
    motorForward(motor, motor_speed);
    
  } else if(motor_speed < 0) {
    motorBackward(motor, motor_speed);
  }
}


