#include <Wire.h>
#include <SDPArduino.h>
#include <SerialCommand.h>
//#include "MPU.h"

const int PIN_LED   = 13;

SerialCommand comm;

void setup()
{  
  Serial.println("Running SDPsetup()");
  // SDP Setup
  SDPsetup();
  
  // Setup LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  Serial.println("Initialising Commands");
  // Initialse the command messenger
  comm.addCommand("RUN",   cmd_RUN);
  comm.addCommand("PING",  cmd_PING);
  comm.addCommand("LED",   cmd_LED);
  comm.addCommand("KICK",  cmd_KICK);
  comm.addCommand("CATCH", cmd_CATCH);

#ifdef HAS_MPU6050 
  Serial.println("Initialising MPU");
  // Initialise MPU
  MPU_setup(&comm);
#endif

  Serial.println("STARTED");
}

/* Get the next argument of the curret command, and convert it to an int */
int getInt(int default_value)
{
    char* arg = comm.next();
    if(arg == NULL) {
        return default_value;
    } else {
        return atoi(arg);
    }
}


/* Control the LED */
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
  Serial.println("DONE");
}

void cmd_PING()
{
  Serial.println("PONG");
  Serial.println("DONE");
}

/* Movement Commands */
void cmd_RUN()
{
  int motor1 = getInt(0);
  int motor2 = getInt(0);
  int motor3 = getInt(0);
  
  doRun(motor1, motor2, motor3);
  Serial.println("DONE");
}

void doRun(int motor1, int motor2, int motor3)
{
  doRunMotor(0, motor1);
  doRunMotor(1, motor2);
  doRunMotor(2, motor3);
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

/* Kicker Commands */
#define MOTOR_KICKER (3)

#define KICKER_RUNNING_TIME (400)

#define KICKER_DEFAULT_SCALE (100)

#define KICKER_STATE_STOPPED  (0x01)
#define KICKER_STATE_KICKING  (0x02)
#define KICKER_STATE_CATCHING (0x03)

byte kicker_state = KICKER_STATE_STOPPED;
unsigned long kicker_stop_time = 0L;

void cmd_KICK()
{
    char* arg = comm.next();
    int scale = KICKER_DEFAULT_SCALE;
    if(arg != NULL) {
        scale = atoi(arg);
    }
    
    if(scale <= 40) {
      scale = KICKER_DEFAULT_SCALE;
    }
  
    kicker_stop_time = millis() + (KICKER_RUNNING_TIME * (100/scale));
    motorForward(MOTOR_KICKER, scale);
    kicker_state = KICKER_STATE_KICKING;
    Serial.println("DONE");
}

void cmd_CATCH()
{
    char* arg = comm.next();
    int scale = KICKER_DEFAULT_SCALE;
    if(arg != NULL) {
        scale = atoi(arg);
    }
    
    if(scale <= 40) {
      scale = KICKER_DEFAULT_SCALE;
    }
  
    kicker_stop_time = millis() + (KICKER_RUNNING_TIME * (100/scale));
    motorBackward(MOTOR_KICKER, 100);
    kicker_state = KICKER_STATE_KICKING;
    Serial.println("DONE");
}

void stop_kicker()
{
    motorStop(MOTOR_KICKER);
    kicker_state = KICKER_STATE_STOPPED;
}

void loop()
{
  /* Process serial commands */
  comm.readSerial();
  
  /* Check the kicker state */
  if(kicker_state != KICKER_STATE_STOPPED) {
      if(millis() >= kicker_stop_time) {
          stop_kicker();
      }
  }
  
#ifdef HAS_MPU6050
    MPU_service();
#endif
}


