#ifndef _MOTORS_C_C
#define _MOTORS_C_C

#define ARM_CONTROLLER      E_CONTROLLER_MASTER
#define DRIVE_CONTROLLER    E_CONTROLLER_MASTER
#define CLAW_CONTROLLER     E_CONTROLLER_MASTER

#define ARM_UP_BTN          E_CONTROLLER_DIGITAL_L1
#define ARM_DN_BTN          E_CONTROLLER_DIGITAL_L2
#define DRIVE_LEFT_STICK    E_CONTROLLER_ANALOG_LEFT_Y
#define DRIVE_RIGHT_STICK   E_CONTROLLER_ANALOG_RIGHT_Y
#define DRIVE_LEFT_STRAFE_STICK     E_CONTROLLER_ANALOG_LEFT_X
#define DRIVE_RIGHT_STRAFE_STICK    E_CONTROLLER_ANALOG_RIGHT_X
#define CLAW_OPEN_BTN       E_CONTROLLER_DIGITAL_R2
#define CLAW_CLOSE_BTN      E_CONTROLLER_DIGITAL_R1
#define CLAW_CW_BTN         E_CONTROLLER_DIGITAL_B
#define CLAW_CCW_BTN        E_CONTROLLER_DIGITAL_A

#define ARM_LEFT_PORT           20
#define ARM_RIGHT_PORT          10
#define DRIVE_LEFT_FRONT_PORT   10
#define DRIVE_LEFT_REAR_PORT    20
#define DRIVE_RIGHT_FRONT_PORT  9
#define DRIVE_RIGHT_REAR_PORT   19
#define CLAW_ROTATE_PORT        9
#define CLAW_PORT               8

#define DRIVE_FORWARD  1
#define DRIVE_BACKWARD -DRIVE_FORWARD
#define ARM_UP   1
#define ARM_DOWN -ARM_UP
#define CLAW_OPEN  1
#define CLAW_CLOSE -CLAW_OPEN
#define CLAW_CW  1
#define CLAW_CCW -CLAW_CW


void armSpeed(int speed);
void leftDrive(int speed);
void rightDrive(int speed);
void clawRotate(int direction);
void clawSpeed(int speed);

#include "../real_src/Motors_C.c"
#endif // _MOTORS_C_C
