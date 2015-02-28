#include <Wire.h>
#include <SDPArduino.h>
#include <SerialCommand.h>
#include "config.h"
#include "MPU.h"
#include "kicker.h"

SerialCommand comm;

void setup()
{  
  // SDP Setup
  SDPsetup();
  Serial.println("Hello! Running setup...");
  
  // Setup LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  // Initialse the command messenger
  comm.addCommand("RUN",     cmd_RUN);
  comm.addCommand("PING",    cmd_PING);
  comm.addCommand("KICK",    cmd_KICK);
  comm.addCommand("CATCH",   cmd_CATCH);
  comm.addCommand("RELEASE", cmd_RELEASE);

#ifdef HAS_MPU6050 
  // Setup MPU
  MPU_setup(&comm);
#endif

  Serial.println("STARTED");
}

void loop()
{
    /* Process serial commands */
    comm.readSerial();
    
    service_kicker();
    service_catcher(); 
  
#ifdef HAS_MPU6050
    MPU_service();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Utilities
////////////////////////////////////////////////////////////////////////////////

/* Get the next argument of the current command, and convert it to an int */
int getInt(int default_value)
{
    char* arg = comm.next();
    if(arg == NULL) {
        return default_value;
    } else {
        return atoi(arg);
    }
}

void cmd_PING()
{
    Serial.println("PONG");
    Serial.println("DONE");
}


////////////////////////////////////////////////////////////////////////////////
// Movement
////////////////////////////////////////////////////////////////////////////////

void cmd_RUN()
{
    int motor1 = getInt(0); // All motors default to 0/stop
    int motor2 = getInt(0);
    int motor3 = getInt(0);
    
    runMotor(MOTOR_MOTOR1, motor1);
    runMotor(MOTOR_MOTOR2, motor2);
    runMotor(MOTOR_MOTOR3, motor3);
    
    Serial.println("DONE");
}

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

////////////////////////////////////////////////////////////////////////////////
// Catching
////////////////////////////////////////////////////////////////////////////////

#define CATCHER_RUNNING_TIME  (400)
#define CATCHER_DEFAULT_SCALE (100)

#define CATCHER_STATE_STOPPED   (0x01)
#define CATCHER_STATE_CATCHING  (0x02)
#define CATCHER_STATE_RELEASING (0x03)

byte catcher_state = CATCHER_STATE_STOPPED;
unsigned long catcher_stop_time = 0L;

void cmd_CATCH()
{
    int scale = getInt(CATCHER_DEFAULT_SCALE);   
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
  
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorForward(MOTOR_CATCHER, 100);
    catcher_state = KICKER_STATE_KICKING;
    
    Serial.println("DONE");
}

void cmd_RELEASE()
{
    int scale = getInt(CATCHER_DEFAULT_SCALE);
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
    
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorBackward(MOTOR_CATCHER, scale);
    catcher_state = KICKER_STATE_RETURNING;
    
    Serial.println("DONE");
}

void service_catcher() {
    /* Check the catcher state */
    if(catcher_state != CATCHER_STATE_STOPPED) {
        if(millis() >= catcher_stop_time) {
            motorStop(MOTOR_KICKER);
            catcher_state = CATCHER_STATE_STOPPED;
        }
    }
}


