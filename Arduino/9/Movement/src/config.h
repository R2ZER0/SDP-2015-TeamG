/* config.h */
#ifndef _CONFIG_H_
#define _CONFIG_H_

#define PIN_LED 13

// 0 = Yaw, 1 = Pitch, 2 = Roll - depends on the orientation of MPU?
#define TURN_AXIS (0)

// in milliseconds
#define STATE_MESSAGE_INTERVAL (120)

// Uncomment to enable MPU trace messages
//#define MPU_DEBUG (true)

#define MPU_OFFSET_ACELX (-6476)
#define MPU_OFFSET_ACELY (-3160)
#define MPU_OFFSET_ACELZ (1117)
#define MPU_OFFSET_GYROX (65)
#define MPU_OFFSET_GYROY (16)
#define MPU_OFFSET_GYROZ (-6)

// Currently set to +/- 1.5 degrees
#define TURN_ACCEPTABLE_RANGE (0.017453*1.5)

// Wheel PID parameters
#define WHEELS_UPDATE_INTERVAL (200.0)
#define WHEELS_KP 1.0
#define WHEELS_KI 0.0
#define WHEELS_KD 0.5
#define WHEELS_POWER_MAX (100)
#define WHEELS_POWER_MIN (30)
#define WHEELS_DEADZONE_SIZE 5.0

#define NUM_MOTORS 4
#define NUM_WHEELS NUM_MOTORS
#define MOTOR_MOTOR1 2
#define MOTOR_MOTOR2 3
#define MOTOR_MOTOR3 0
#define MOTOR_MOTOR4 5
#define MOTOR_KICKER 4
#define MOTOR_CATCHER 1

#define KICKER_RUNNING_TIME  (400)
#define KICKER_RETURNING_TIME (800)
#define KICKER_DEFAULT_SCALE (100)

#define CATCHER_RUNNING_TIME  (550)
#define CATCHER_DEFAULT_SCALE (100)

#define ROTARY_SLAVE_ADDRESS 5
#define ROTARY_COUNT 6

#define TURN_QUICKSTART_TIME (150)

#endif /* _CONFIG_H_ */