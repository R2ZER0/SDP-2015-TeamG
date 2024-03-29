////////////////////////////////////////////////////////////////////////////////
// Command
////////////////////////////////////////////////////////////////////////////////
#include "command.h"
#include <Arduino.h>

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

// State message sending timeout
static unsigned long next_message_time = 0L;

// Buffer for reading/writing messages
#define BUFFER_SIZE 128
static char send_buffer[BUFFER_SIZE];
static char recv_buffer[BUFFER_SIZE];
static int  recv_buffer_i = 0;

void run_movement_command(unsigned long id, char cmd, int arg1, int arg2)
{
    movement_on_new_command(cmd, arg1, arg2);
        
    movement_command_id = id;
    movement_command = cmd;
}

void run_kicker_command(unsigned long id, char cmd, int arg1)
{
    kicker_on_new_command(cmd, arg1);
    
    kicker_command_id = id;
    kicker_command = cmd;
}

void run_catcher_command(unsigned long id, char cmd, int arg1)
{
    catcher_on_new_command(cmd, arg1);
    
    catcher_command_id = id;
    catcher_command = cmd;
}

void send_state_message(void)
{
    snprintf(&send_buffer[0], BUFFER_SIZE,
        "(%lu %c %d %lu %c %d %lu %c %d)\n",
        movement_command_id, movement_command, movement_command_fin,
        kicker_command_id, kicker_command, kicker_command_fin,
        catcher_command_id, catcher_command, catcher_command_fin
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
        int move_arg1, move_arg2, kick_arg1, catch_arg1;
    
        int result = sscanf(&recv_buffer[0], "(%lu %c %d %d %lu %c %d %lu %c %d)",
            &move_cmd_id, &move_cmd, &move_arg1, &move_arg2,
            &kick_cmd_id, &kick_cmd, &kick_arg1,
            &catch_cmd_id, &catch_cmd, &catch_arg1
        );
        
        // Make sure we got a full match
        if(result == 10) {
            //Serial.println("Matching Result");
            
            if(move_cmd_id != movement_command_id) {
                run_movement_command(move_cmd_id, move_cmd, move_arg1, move_arg2);
            }
            
            if(kick_cmd_id != kicker_command_id) {
                run_kicker_command(kick_cmd_id, kick_cmd, kick_arg1);
            }
            
            if(catch_cmd_id != catcher_command_id) {
                run_catcher_command(catch_cmd_id, catch_cmd, catch_arg1);
            }
            
        } else {
            //Serial.print("Got non-matching result ");
            //Serial.println(result);
        }
    }
}

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
