////////////////////////////////////////////////////////////////////////////////
// Movement
////////////////////////////////////////////////////////////////////////////////
#include "movement.h"
#include "Angle.h"
#include <Arduino.h>
#include <SDPArdino.h>
#include "config.h"
#include "command.h"

int current_command = MOVEMENT_COMMAND_STOP;
Angle targetAngle();

void on_new_command(int cmd, float dir, int spd)
{
    // TODO
}

void setup_movement()
{
    stopAllMotors();
}

void service_movement()
{
    command_sethook_movement(&on_new_command);
}