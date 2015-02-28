#include <Wire.h>
#include <SDPArduino.h>
#include "MPU.h"
#include "config.h"
#include "kicker.h"

void setup()
{
  // SDP Setup
  SDPsetup();
  
  // Setup LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  

#ifdef HAS_MPU6050 
  // Setup MPU
  MPU_setup();
#endif

  Serial.println("STARTED");
}

void loop()
{
    
    service_kicker();
    service_catcher(); 
  
#ifdef HAS_MPU6050
    MPU_service();
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Movement
////////////////////////////////////////////////////////////////////////////////

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

void catcher_catch(int scale)
{
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
  
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorForward(MOTOR_CATCHER, 100);
    catcher_state = CATCHER_STATE_CATCHING;
}

void catcher_release(int scale)
{
    if(scale <= 40) {
        scale = CATCHER_DEFAULT_SCALE;
    }
    
    catcher_stop_time = millis() + (CATCHER_RUNNING_TIME * (100/scale));
    motorBackward(MOTOR_CATCHER, scale);
    catcher_state = CATCHER_STATE_RELEASING;
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


