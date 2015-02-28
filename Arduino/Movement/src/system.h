/* system.h */
#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#define SYSTEM_STATE_STARTING       (0x01)
#define SYSTEM_STATE_INITIALISING   (0x02)
#define SYSTEM_STATE_READY          (0x03)
#define SYSTEM_STATE_MOVING         (0x04)
#define SYSTEM_STATE_TURNING        (0x05)

void setup_system(void);
void service_system(void);

#endif /* _SYSTEM_H_ */