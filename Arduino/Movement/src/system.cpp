////////////////////////////////////////////////////////////////////////////////
// System
////////////////////////////////////////////////////////////////////////////////
#include "system.h"
#include <Arduino.h>
#include <SDPArduino.h>
#include "config.h"
#include "MPU.h"
#include "kicker.h"
#include "catcher.h"

static char system_state = SYSTEM_STATE_STARTING;

char system_get_state(void)
{
    return system_state;
}

void setup_system(void)
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

void service_system(void)
{
    service_kicker();
    service_catcher(); 
  
#ifdef HAS_MPU6050
    MPU_service();
#endif
}