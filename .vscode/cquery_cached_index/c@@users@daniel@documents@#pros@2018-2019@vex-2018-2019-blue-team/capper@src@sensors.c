#include "../include/Sensors.h"
#include "../include/main.h"
/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
void moveIn(float inches) {
/*  float deg_per_inch = 360 / (PI * WHEEL_DIAMETER);
  float targetDegrees = inches * deg_per_inch;
  int startPositionLeft = motor_get_position(LEFT_MOTOR);
  int startPositionRight = motor_get_position(RIGHT_MOTOR);
  while(motor_get_position(LEFT_MOTOR) - startPositionLeft < targetDegrees) {
    float speed = .5;
    int leftPos = motor_get_position(LEFT_MOTOR) - startPositionLeft;
    int rightPos = motor_get_position(RIGHT_MOTOR) - startPositionRight;
    if(leftPos > rightPos) {
      leftWheels(WHEELS_FORWARD * .5);
      rightWheels(WHEELS_FORWARD * .25);
    } else if(leftPos < rightPos) {
      leftWheels(WHEELS_FORWARD * .25);
      rightWheels(WHEELS_FORWARD * .5);
    } else {
      leftWheels(WHEELS_FORWARD * .5);
      rightWheels(WHEELS_FORWARD * .5);

    }

  }
  leftWheels(0);
  rightWheels(0);*/
}
/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg) {

}
