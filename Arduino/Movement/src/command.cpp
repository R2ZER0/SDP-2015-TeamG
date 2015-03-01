////////////////////////////////////////////////////////////////////////////////
// Command
////////////////////////////////////////////////////////////////////////////////
#include "command.h"
#include <Arduino.h>
#include "config.h"
#include "MPU.h"

#define MOVEMENT_COMMAND_MOVE 'M'
#define MOVEMENT_COMMAND_TURN 'T'
#define MOVEMENT_COMMAND_STOP 'S'

#define KICKER_COMMAND_KICK 'K'
#define KICKER_COMMAND_IDLE 'I'

#define CATCHER_COMMAND_CATCH   'C'
#define CATCHER_COMMAND_RELEASE 'R'
#define CATCHER_COMMAND_IDLE    'I'

// Command IDs
static unsigned long movement_command_id = 0L;
static unsigned long kicker_command_id = 0L;
static unsigned long catcher_command_id = 0L;

// Commands being executed
static int movement_command = MOVEMENT_COMMAND_STOP;
static int kicker_command = KICKER_COMMAND_IDLE;
static int catcher_command = CATCHER_COMMAND_IDLE;

// Movement command args
static float movement_direction = 0.0f;
static int movement_speed = 0;

// State message sending timeout
static unsigned long next_message_time = 0L;

// Buffer for reading/writing messages
static char buffer[256]; // this is probably bigger than it needs to be

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

// A simple encoding for floats
int f2i(float f) { return (int)(f * 1024); }
float i2f(int i) { return ((float) i)/1024.0f; }

void send_state_message(void)
{
    snprintf(&buffer[0], sizeof(buffer)/sizeof(buffer[0]),
        "[ "
        "M I%ld C%c D%d S%d F%d "
        "K I%ld C%c F%d "
        "C I%ld C%c F%d "
        "S D%d "
        "]",
        
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