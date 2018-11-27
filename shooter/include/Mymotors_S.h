#ifndef _MYMOTORS_H_
#define _MYMOTORS_H_

#define MOTOR_FRONT_LEFT 1
#define MOTOR_FRONT_RIGHT 3
#define MOTOR_BACK_LEFT 2
#define MOTOR_BACK_RIGHT 4
#define MOTOR_CATAPULT_LEFT 5
#define MOTOR_CATAPULT_RIGHT 6
#define MOTOR_INTAKE 7
#define WHEELS_FORWARD 127
#define WHEELS_BACKWARD -127
#define WHEEL_DIAMETER 4
#define PI 3.1415


#include "../../include/PID.h"


//void initMotors(int motor, int gearset, bool reversed);
//PID_array_t initDrive(double Kp, double Ki, double Kd);

void initPID();

void moveIn(double left, double right);

void turnDeg(double deg);

/* Function:		leftWheels
 * Purpose:			moves the left side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
//void myleftWheels(float speed);

/* Function:		rightWheels
 * Purpose:			moves the right side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
//void myrightWheels(float speed);
#include "Mymotors_S.c"
#endif // _MYMOTORS_H_
