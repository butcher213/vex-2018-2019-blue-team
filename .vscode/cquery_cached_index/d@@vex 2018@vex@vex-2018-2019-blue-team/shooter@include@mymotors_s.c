#include "main_S.h"
#include "Mymotors_S.h"
/* Function:		leftWheels
 * Purpose:			moves the left side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void myleftWheels(float speed) {
  motor_move(MOTOR_LEFT, speed);
}

/* Function:		rightWheels
 * Purpose:			moves the right side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void myrightWheels(float speed) {
  motor_move(MOTOR_RIGHT, speed);
}
