#ifndef _MPU_H_
#define _MPU_H_

#define HAS_MPU6050 1

void MPU_setup(SerialCommand* _comm);
void MPU_service();

#endif /* _MPU_H_ */