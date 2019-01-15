#ifndef _MOTORS_C_C
#define _MOTORS_C_C

#define ARM_CONTROLLER      CONTROLLER_MASTER
#define DRIVE_CONTROLLER    CONTROLLER_MASTER
#define CLAW_CONTROLLER     CONTROLLER_MASTER

#define ARM_UP_BTN
#define DRIVE_LEFT_STICK
#define DRIVE_RIGHT_STICK
#define CLAW_BTN
#define CLAW_ROTATE_BTN

#define ARM_LEFT_PORT           20
#define ARM_RIGHT_PORT          10
#define DRIVE_LEFT_FRONT_PORT   11
#define DRIVE_LEFT_REAR_PORT    12
#define DRIVE_RIGHT_FRONT_PORT  1
#define DRIVE_RIGHT_REAR_PORT   2
#define CLAW_ROTATE_PORT        9
#define CLAW_PORT               8

#define DRIVE_FORWARD
#define DRIVE_BACKWARD -DRIVE_FORWARD
#define ARM_UP
#define ARM_DOWN -ARM_UP
#define CLAW_OPEN
#define CLAW_CLOSE -CLAW_OPEN
#define CLAW_CCW
#define CLAW_CW -CLAW_CCW


void armSpeed(int speed);
void leftDrive(int speed);
void rightDrive(int speed);
void clawRotate(int direction);
void clawSpeed(int speed);

#include "../real_src/Motors_C.c"
#endif // _MOTORS_C_C
