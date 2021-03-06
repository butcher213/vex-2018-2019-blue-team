#include "../include/Sensors_C.h"
#include "../include/main_C.h"

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
void rotateDeg(float targetDeg,PID_properties_t leftMotors, PID_properties_t rightMotors) {
	float angle = 0
	float angleTarget = (algle*((16+(5/8))/2)/2)
	leftMotors = generateMovedPID(leftMotors,angleTarget);
	rightMotors = generateMovedPID(rightMotors,angleTarget);
	while ! (atTarget(leftMotors)){
		leftMotors = generateNextPID(leftMotors);
		rightMotors = generateNextPID(rightMotors);
	}
	delay(1000);
		
	
	

}

void getCap(){

}

void putOnPole() {

}

void putOnBigPole() {

}

void flipCap(){

}

void dropCap(){

}
