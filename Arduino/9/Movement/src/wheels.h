/* wheels.h */
#ifndef _WHEELS_H_
#define _WHEELS_H_

void wheels_set_target_speeds(double* speed);
void wheels_stop();
double wheels_get_speed(int wheel);
void wheels_control_enabled(bool enabled);

void setup_wheels();
void service_wheels();

#endif /* _WHEELS_H_ */