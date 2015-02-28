#ifndef _MPU_H_
#define _MPU_H_

#define HAS_MPU6050 1

void MPU_setup();
void MPU_service();

extern float getAngle(int i);

#endif /* _MPU_H_ */