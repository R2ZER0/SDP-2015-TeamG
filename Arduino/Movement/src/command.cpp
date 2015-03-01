////////////////////////////////////////////////////////////////////////////////
// Command
////////////////////////////////////////////////////////////////////////////////
#include "command.h"
#include <Arduino.h>
#include "config.h"
#include "MPU.h"

// Command IDs
static unsigned long movement_command_id = 0L;
static unsigned long kicker_command_id = 0L;
static unsigned long catcher_command_id = 0L;

// Commands being executed
static int movement_command = MOVEMENT_COMMAND_STOP;
static int kicker_command = KICKER_COMMAND_IDLE;
static int catcher_command = CATCHER_COMMAND_IDLE;

// Wether they have finished
static bool movement_command_fin = false;
static bool kicker_command_fin = false;
static bool catcher_command_fin = false;

// Movement command args
static float movement_direction = 0.0f;
static int movement_speed = 0;

// State message sending timeout
static unsigned long next_message_time = 0L;

// Hook functions
movement_hook_t movement_hook = NULL;
kicker_hook_t   kicker_hook   = NULL;
catcher_hook_t  catcher_hook  = NULL;

// Buffer for reading/writing messages
static char buffer[256]; // this is probably bigger than it needs to be

void run_movement_command(unsigned long id, int cmd, float dir, int speed)
{
    (*movement_hook)(cmd, dir, speed);
    
    movement_command_id = id;
    movement_command = cmd;
    movement_direction = dir;
    movement_speed = speed;
}

void run_kicker_command(unsigned long id, int cmd)
{
    (*kicker_hook)(cmd, 100);
    
    kicker_command_id = id;
    kicker_command = cmd;
}

void run_catcher_command(unsigned long id, int cmd)
{
    (*catcher_hook)(cmd, 100);
    
    catcher_command_id = id;
    catcher_command = cmd;
}

// A simple encoding for floats
int f2i(float f) { return (int)(f * 1024); }
float i2f(int i) { return ((float) i)/1024.0f; }

void send_state_message(void)
{
    snprintf(&buffer[0], sizeof(buffer)/sizeof(buffer[0]),
        "(%lu %c %d %d %d %lu %c %d %lu %c %d %d)",        
        movement_command_id, movement_command, f2i(movement_direction),
        movement_speed, 0,
        kicker_command_id, kicker_command, 0,
        catcher_command_id, catcher_command, 0,
        f2i(getAngle())        
    );
    
    Serial.println((char*)(&buffer[0]));
}

void process_state_message(void)
{
    unsigned long move_cmd_id, kick_cmd_id, catch_cmd_id;
    int move_cmd, kick_cmd, catch_cmd;
    float move_dir;
    int move_speed, kick_speed, catch_speed;
    
    // Make sure we've got a proper message
    buffer[0] = Serial.read();
    if(buffer[0] != '(') {
        // If not, throw it away
        while(Serial.available() > 0) { Serial.read(); }
        return;
    }
    
//     message = "({0} {1} {2} {3} {4} {5} {6} {7} {8} {9})".format(
//                 self.move.idx, self.move.cmd, f2i(self.move.dir), self.move.spd,
//                 self.kick.idx, self.kick.cmd, self.kick.spd,
//                 self.catch.idx, self.catch.cmd, self.catch.spd
//     )
    
    int i = 0;
    do {
        ++i;
        buffer[i] = Serial.read();
    } while(buffer[i] != ')');
    buffer[i+1] = '\0';
    
    int move_dir_tmp;
    sscanf(&buffer[0], "(%lu %c %d %d %lu %c %d %lu %c %d)",
           &move_cmd_id, &move_cmd, &move_dir_tmp, &move_speed,
           &kick_cmd_id, &kick_cmd, &kick_speed,
           &catch_cmd_id, &catch_cmd, &catch_speed
    );
    
    move_dir = i2f(move_dir_tmp);
    
    if(move_cmd_id != movement_command_id) {
        if(move_cmd == 'M' || move_cmd == 'T' || move_cmd == 'S') {
            run_movement_command(move_cmd_id, move_cmd, move_dir, move_speed);
        }
    }
    
    if(kick_cmd_id != kicker_command_id) {
        if(kick_cmd == 'K' || kick_cmd == 'I') {
            run_kicker_command(kick_cmd_id, kick_cmd);
        }
    }
    
    if(catch_cmd_id != catcher_command_id) {
        if(catch_cmd == 'C' || catch_cmd == 'R' || catch_cmd == 'I') {
            run_catcher_command(catch_cmd_id, catch_cmd);
        }
    }
}

void command_sethook_movement(movement_hook_t hook) { movement_hook = hook; }
void command_sethook_kicker(kicker_hook_t hook)     { catcher_hook = hook; }
void command_sethook_catcher(catcher_hook_t hook)   { kicker_hook = hook; }

void command_finished_movement(void) { movement_command_fin = true; }
void command_finished_kicker(void)   { kicker_command_fin = true; }
void command_finished_catcher(void)  { catcher_command_fin = true; }

void setup_command(void)
{
    // TODO
}

void service_command(void)
{
    if(Serial.available() > 0) {
        process_state_message();
    }
    
    if(millis() >= next_message_time) {
        send_state_message();
        next_message_time = millis() + STATE_MESSAGE_INTERVAL;
    }
}