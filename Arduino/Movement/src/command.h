/* command.h */
#ifndef _COMMAND_H_
#define _COMMAND_H_

#define MOVEMENT_COMMAND_MOVE 'M'
#define MOVEMENT_COMMAND_TURN 'T'
#define MOVEMENT_COMMAND_STOP 'S'

#define KICKER_COMMAND_KICK 'K'
#define KICKER_COMMAND_IDLE 'I'

#define CATCHER_COMMAND_CATCH   'C'
#define CATCHER_COMMAND_RELEASE 'R'
#define CATCHER_COMMAND_IDLE    'I'

typedef void (*movement_hook_t)(int, float, int);
typedef void (*kicker_hook_t)(int, int);
typedef void (*catcher_hook_t)(int, int);

void command_sethook_movement(movement_hook_t hook);
void command_sethook_kicker(kicker_hook_t hook);
void command_sethook_catcher(catcher_hook_t hook);

void command_finished_movement(void);
void command_finished_kicker(void);
void command_finished_catcher(void);

void setup_command(void);
void service_command(void);

#endif _COMMAND_H_