#include "main.h"
//#include "Motors2.h"
/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
 #define PI 3.1415
void moveIn(float inches) {
  double rightpos = 0;
  double target = inches * PI * 4;
  motor_set_zero_position(2,0);
  while(rightpos / 360 < 12 * PI * 4) {
    rightpos =  motor_get_position(2);
    printf("value: %.2f\n", rightpos);
    double leftpos = motor_get_position(1);
    if(leftpos > rightpos) {
      motor_move(2, 10);
      motor_move(1,10 * .9);
    } else {
      motor_move(2, 10);
      motor_move(1,10 * .9);
    }
  }
  motor_move(2,0);
  motor_move(1,0);
}
/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg) {

}
