#include <Wire.h>
#include <SDPArduino.h>
#include "MPU.h"
#include "config.h"
#include "kicker.h"
#include "catcher.h"

#define SYSTEM_STATE_STARTING       (0x01)
#define SYSTEM_STATE_INITIALISING   (0x02)
#define SYSTEM_STATE_READY          (0x03)
#define SYSTEM_STATE_MOVING         (0x04)
#define SYSTEM_STATE_TURNING        (0x05)

static char system_state = SYSTEM_STATE_STARTING;

void service_system(void)
{
    // TODO
}


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

  system_state = SYSTEM_STATE_STARTING;
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