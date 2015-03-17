/* command.h */
#ifndef _COMMAND_H_
#define _COMMAND_H_

// Message interval in ms
#define STATE_MESSAGE_INTERVAL (120)

// FIXME: Redefine these
#define MOVEMENT_COMMAND_MOVE 'M'
#define MOVEMENT_COMMAND_TURN 'T'
#define MOVEMENT_COMMAND_STOP 'S'

#define KICKER_COMMAND_KICK 'K'
#define KICKER_COMMAND_IDLE 'I'

#define CATCHER_COMMAND_CATCH   'C'
#define CATCHER_COMMAND_RELEASE 'R'
#define CATCHER_COMMAND_IDLE    'I'

void command_finished_movement(void);
void command_finished_kicker(void);
void command_finished_catcher(void);

void setup_command(void);
void service_command(void);

// Implement these in your sketch!
void movement_on_new_command(char cmd, int arg1, int arg2);
void kicker_on_new_command(char cmd, int arg1);
void catcher_on_new_command(char cmd, int arg1);

#endif _COMMAND_H_