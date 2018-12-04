// #include "../include/Sensors_C.h"
// #include "../../include/PID.h"
#include "../include/main_C.h"
#ifdef _PID_H_
#warning "SENSORS: PID DEFINED"
#endif

PID_properties_t leftWheels, rightWheels;

void initializePIDs() {
    int leftWheelPorts[] = {11, 12};
    int rightWheelPorts[] = {1, 2};
    float driveKp = 0.2;
    float driveKi = 0.00000035;
    float driveKd = 0.0001;
    leftWheels  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, 20);
    rightWheels = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, 20);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_COUNTS);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}

void moveIn(float inches) {


/*  float deg_per_inch = 360 / (PI * WHEEL_DIAMETER);
  float targetDegrees = inches * deg_per_inch;
  int startPositionLeft = motor_get_position(LEFT_MOTOR);
  int startPositionRight = motor_get_position( RIGHT_MOTOR);
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

void rotateTo(float targetDeg) {

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
