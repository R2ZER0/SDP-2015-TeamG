/* config.h */
#ifndef _CONFIG_H_
#define _CONFIG_H_

#define PIN_LED 13

// 0 = Yaw, 1 = Pitch, 2 = Roll - depends on the orientation of MPU?
#define TURN_AXIS (0)

// in milliseconds
#define STATE_MESSAGE_INTERVAL (50)

// Uncomment to enable MPU trace messages
//#define MPU_DEBUG (true)

#define MPU_OFFSET_ACELX (-6476)
#define MPU_OFFSET_ACELY (-3160)
#define MPU_OFFSET_ACELZ (1117)
#define MPU_OFFSET_GYROX (65)
#define MPU_OFFSET_GYROY (16)
#define MPU_OFFSET_GYROZ (-6)

// Currently set to +/- 1 degree
#define TURN_ACCEPTABLE_RANGE (0.017453)

#define NUM_MOTORS 4
#define MOTOR_MOTOR1  1
#define MOTOR_MOTOR2  3
#define MOTOR_MOTOR3  0
#define MOTOR_MOTOR4  2
#define MOTOR_KICKER  4
#define MOTOR_CATCHER 5

#define KICKER_RUNNING_TIME  (400)
#define KICKER_RETURNING_TIME (800)
#define KICKER_DEFAULT_SCALE (100)

#define CATCHER_RUNNING_TIME  (400)
#define CATCHER_DEFAULT_SCALE (100)

#endif /* _CONFIG_H_ */