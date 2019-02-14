#ifndef _MYMOTORS_H_
#define _MYMOTORS_H_

#define MOTOR_FRONT_LEFT 11
#define MOTOR_FRONT_RIGHT 13
#define MOTOR_BACK_LEFT 2
#define MOTOR_BACK_RIGHT 13
#define MOTOR_CATAPULT_LEFT 5
#define MOTOR_CATAPULT_RIGHT 6
#define MOTOR_INTAKE 7
#define MOTOR_FLAPPER 8
#define MOTOR_BELT 12
#define WHEELS_FORWARD 127
#define WHEELS_BACKWARD -127
#define WHEEL_DIAMETER 4
#define PI 3.1415
#define TILE_LENGTH 22.5

#include "../../include/PID.h"


//void initMotors(int motor, int gearset, bool reversed);
//PID_array_t initDrive(double Kp, double Ki, double Kd);

void initPID();

void moveIn(double left, double right);

void turnDeg(double deg);

void launchCatapult(void);

void stopDriveMotors(void);

void spinIntake(double multiplier);
#include "Mymotors_S.c"
#endif // _MYMOTORS_H_
