/* kicker.h */

#ifndef _KICKER_H_
#define _KICKER_H_
#include "config.h"

#define KICKER_STATE_STOPPED   (0x01)
#define KICKER_STATE_KICKING   (0x02)
#define KICKER_STATE_RETURNING (0x03)

char kicker_get_state(void);
void kicker_kick(int kicker_scale);
void setup_kicker(void);
void service_kicker(void);

#endif /* _KICKER_H_ */