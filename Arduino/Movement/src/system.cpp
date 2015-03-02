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
#include "command.h"
#include "movement.h"

static char system_state = SYSTEM_STATE_STARTING;

char system_get_state(void)
{
    return system_state;
}

void setup_system(void)
{    
    // Setup LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    system_state = SYSTEM_STATE_STARTING;
    
    delay(100); // flash the LED
    digitalWrite(PIN_LED, LOW);
    
    // Now setup things
    // SDP Setup
    SDPsetup();
    
#ifdef HAS_MPU6050 
    // Setup MPU
    MPU_setup();
#endif
    
    setup_command();
    setup_movement();
    setup_kicker();
    
    digitalWrite(PIN_LED, HIGH);
}

void service_system(void)
{
    service_command();
    service_kicker();
    service_catcher(); 
#ifdef HAS_MPU6050
    MPU_service();
#endif
    service_movement();
}