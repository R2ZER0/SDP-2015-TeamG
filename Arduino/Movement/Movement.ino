#include <Wire.h>
#include <SDPArduino.h>
#include <SerialCommand.h>

const int PIN_LED   = 13;
const int PIN_RADIO = 8;

SerialCommand comm;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  delay(5000);
  
  digitalWrite(13, HIGH);
  
  // SDP Setup
  SDPsetup();
  
  // Initialse the command messenger
  comm.addCommand("PING", cmd_PING);
  comm.addCommand("RUN", cmd_RUN);
  
  Serial.println("STARTED");
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

#define HEARTBEAT_RATE 10000
int heartbeatticks = HEARTBEAT_RATE;
int heartbeatcount = 0;

void loop()
{
  if(heartbeatticks-- < 0) {
    Serial.print("HB ");
    Serial.println(heartbeatcount++);
    heartbeatticks = HEARTBEAT_RATE;
  }
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


