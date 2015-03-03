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
#define BUFFER_SIZE 128
static char send_buffer[BUFFER_SIZE];
static char recv_buffer[BUFFER_SIZE];
static int  recv_buffer_i = 0;

void movement_on_new_command(char cmd, float dir, int spd);
void run_movement_command(unsigned long id, char cmd, float dir, int speed)
{
    movement_on_new_command(cmd, dir, speed);
        
    movement_command_id = id;
    movement_command = cmd;
    movement_direction = dir;
    movement_speed = speed;
}

void kicker_on_new_command(char cmd, int spd);
void run_kicker_command(unsigned long id, char cmd)
{
    kicker_on_new_command(cmd, 100);
    
    kicker_command_id = id;
    kicker_command = cmd;
}


void catcher_on_new_command(char cmd, int spd);
void run_catcher_command(unsigned long id, char cmd)
{
    catcher_on_new_command(cmd, 100);
    
    catcher_command_id = id;
    catcher_command = cmd;
}

// A simple encoding for floats
int f2i(float f) { return (int)(f * 1024); }
float i2f(int i) { return ((float) i)/1024.0f; }

void send_state_message(void)
{
    snprintf(&send_buffer[0], BUFFER_SIZE,
        "(%lu %c %d %d %lu %c %d %lu %c %d %d)\n",        
        movement_command_id, movement_command, f2i(movement_direction), movement_command_fin,
        kicker_command_id, kicker_command, kicker_command_fin,
        catcher_command_id, catcher_command, catcher_command_fin,
        f2i(getAngle())        
    );
    
    Serial.print((char*)(&send_buffer[0]));
}

void process_state_message(void)
{    
    //Serial.println("Read into buffer");
    
    bool finished_message = false;
    while(Serial.available() > 0) {
        if(recv_buffer_i >= BUFFER_SIZE) {
            // Buffer overflow :(
            recv_buffer_i = 0;
        }
        
        recv_buffer[recv_buffer_i] = Serial.read();
        
        if(recv_buffer[recv_buffer_i] == ')') {
            finished_message = true;
            recv_buffer[recv_buffer_i + 1] = '\0';
            recv_buffer_i = 0;
            break;
        }
        
        ++recv_buffer_i;
    }
    
    if(finished_message) {
        //Serial.print("Got: ");
        //Serial.println(recv_buffer);
        
        unsigned long move_cmd_id, kick_cmd_id, catch_cmd_id;
        char move_cmd, kick_cmd, catch_cmd;
        float move_dir;
        int move_speed, kick_speed, catch_speed;
    
        int move_dir_tmp;
        int result = sscanf(&recv_buffer[0], "(%lu %c %d %d %lu %c %d %lu %c %d)",
            &move_cmd_id, &move_cmd, &move_dir_tmp, &move_speed,
            &kick_cmd_id, &kick_cmd, &kick_speed,
            &catch_cmd_id, &catch_cmd, &catch_speed
        );
        
        // Make sure we got a full match
        if(result == 10) {
            //Serial.println("Matching Result");
            
            move_dir = i2f(move_dir_tmp);
            
            if(move_cmd_id != movement_command_id) {
                if(move_cmd == 'M' || move_cmd == 'T' || move_cmd == 'S') {
                    Serial.println("MOVE");
                    run_movement_command(move_cmd_id, move_cmd, move_dir, move_speed);
                    Serial.println("ENDMOVE");
                }
            }
            
            if(kick_cmd_id != kicker_command_id) {
                if(kick_cmd == 'K' || kick_cmd == 'I') {
                    Serial.println("KICK");
                    run_kicker_command(kick_cmd_id, kick_cmd);
                    Serial.println("ENDKICK");
                }
            }
            
            if(catch_cmd_id != catcher_command_id) {
                if(catch_cmd == 'C' || catch_cmd == 'R' || catch_cmd == 'I') {
                    Serial.println("CATCH");
                    run_catcher_command(catch_cmd_id, catch_cmd);
                    Serial.println("ENDCATCH");
                }
            }
            
        } else {
            //Serial.print("Got non-matching result ");
            //Serial.println(result);
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
    Serial.println("setup_command");
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