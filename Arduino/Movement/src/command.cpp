////////////////////////////////////////////////////////////////////////////////
// Command
////////////////////////////////////////////////////////////////////////////////
#include "command.h"
#include <Arduino.h>
#include "MPU.h"

/*
 * There are two kinds of message, Arduino to Commander, and Commander to
 * Arduino. Both have the same format. The distinction is that C2A
 * messages relay the state we wish the arduino to be in, 
 * 
 * Message Format:
 * "[<MoveCmdID> <MoveCmd> <MoveCmdDir> <MoveCmdSpd> <KickCmdId> <KickCmd> <CatchCmdId> <CatchCmd> <CurrentAngle>]"
 * Where:
 *  <...CmdID>   = id of command being sent. Non-negative integer.
 *  <...Cmd>     = command. Only the uppercase first letter is sent.
 *  <MoveCmdDir> = direction argument (angle in radians, between -PI and PI). Float.
 *  <MoveCmdSpd> = speed argument. Scale from 0 to 100. Integer.
 *  <MoveCmd>    = One of M for Move, T for Turn, or S for Stop.
 *  <KickCmd>    = One of K for Kick, or I for idle.
 *  <CatchCmd>   = One of C for Catch, R for Release, or I for Idle.
 *  <CurrentAngle> = Current angle we believe the robot is facing.
 */

// Command IDs
static unsigned long movement_command_id = 0L;
static unsigned long kicker_command_id = 0L;
static unsigned long catcher_command_id = 0L;

// Commands being executed
static char movement_command = 'S';
static char kicker_command = 'I';
static char catcher_command = 'C';

// Movement command args
static float movement_direction = 0.0f;
static int movement_speed = 0;

void run_movement_command(unsigned long id, char cmd, float dir, int speed)
{
    // TODO
    movement_command_id = id;
    movement_command = cmd;
    movement_direction = dir;
    movement_speed = speed;
}

void run_kicker_command(unsigned long id, char cmd)
{
    // TODO
    kicker_command_id = id;
    kicker_command = cmd;
}

void run_catcher_command(unsigned long id, char cmd)
{
    // TODO
    catcher_command_id = id;
    catcher_command = cmd;
}

void send_state_message(void)
{
    Serial.print("[");
    Serial.print(movement_command_id); Serial.print(' ');
    Serial.print(movement_command); Serial.print(' ');
    Serial.print(movement_direction); Serial.print(' ');
    Serial.print(movement_speed); Serial.print(' ');
    Serial.print(kicker_command_id); Serial.print(' ');
    Serial.print(kicker_command); Serial.print(' ');
    Serial.print(catcher_command_id); Serial.print(' ');
    Serial.print(catcher_command); Serial.print(' ');
    Serial.print(getAngle());
    Serial.println("]");
}

void process_state_message(void)
{
    unsigned long move_cmd_id, kick_cmd_id, catch_cmd_id;
    char move_cmd, kick_cmd, catch_cmd;
    float move_dir;
    int move_speed;
    
    // Make sure we've got a proper message
    if(Serial.read() != '[') {
        // If not, throw it away
        while(Serial.available() > 0) { Serial.read(); }
        return;
    }
    
    // Read the things:
    move_cmd_id = Serial.parseInt();
    Serial.read(); // space
    
    move_cmd = Serial.read();
    Serial.read(); // space
    
    move_dir = Serial.parseFloat();
    Serial.read(); // space
    
    move_speed = Serial.parseInt();
    Serial.read(); // space
    
    kick_cmd_id = Serial.parseInt();
    Serial.read(); // space
    
    kick_cmd = Serial.read();
    Serial.read(); // space
    
    catch_cmd_id = Serial.parseInt();
    Serial.read(); // space
    
    catch_cmd = Serial.read();
    Serial.read(); // space
    
    Serial.parseFloat(); // CurrentAngle
    Serial.read(); // Terminating ]
}

void setup_command(void)
{
    // TODO
}

void service_command(void)
{
    if(Serial.available() > 0) {
        process_state_message();
    }
}