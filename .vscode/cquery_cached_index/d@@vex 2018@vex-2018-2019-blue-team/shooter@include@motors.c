#include "Main.h"
/* Function:		leftWheels
 * Purpose:			moves the left side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void leftWheels(float speed) {
  motor_move(MOTOR_LEFT, speed);
}

/* Function:		rightWheels
 * Purpose:			moves the right side of the robot forward
 * Arguments:	  speed: the speed of the motors
 * Returns:			N/A
 */
void rightWheels(float speed) {
  motor_move(MOTOR_RIGHT, speed);
}
