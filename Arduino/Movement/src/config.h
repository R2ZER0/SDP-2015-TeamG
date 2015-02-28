/* config.h */
#ifndef _CONFIG_H_
#define _CONFIG_H_

//#define MPU_DEBUG (true) // Uncomment to enable MPU trace messages

#define PIN_LED 10

#define TURN_AXIS (0) // 0 = Yaw, 1 = Pitch, 2 = Roll

#define MOTOR_MOTOR1  0
#define MOTOR_MOTOR2  2
#define MOTOR_MOTOR3  3
#define MOTOR_KICKER  1
#define MOTOR_CATCHER 5

#define KICKER_RUNNING_TIME  (400)
#define KICKER_RETURNING_TIME (800)
#define KICKER_DEFAULT_SCALE (100)

#define CATCHER_RUNNING_TIME  (400)
#define CATCHER_DEFAULT_SCALE (100)

#endif /* _CONFIG_H_ */