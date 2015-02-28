/* catcher.h */
#ifndef _CATCHER_H_
#define _CATCHER_H_

#define CATCHER_STATE_STOPPED   (0x01)
#define CATCHER_STATE_CATCHING  (0x02)
#define CATCHER_STATE_RELEASING (0x03)

void catcher_catch(int scale);
void catcher_release(int scale);
void service_catcher(void);

#endif /* _CATCHER_H_*/