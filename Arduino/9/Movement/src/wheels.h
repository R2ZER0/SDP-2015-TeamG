/* wheels.h */
#ifndef _WHEELS_H_
#define _WHEELS_H_

void wheels_set_target_speed(int wheel, double speed);
double wheels_get_speed(int wheel);

void setup_wheels();
void service_wheels();

#endif /* _WHEELS_H_ */