#include "../include/main_C.h"

PID_properties_t wheelsLeft, wheelsRight;
int leftWheelPorts[] = {10, 9};
int rightWheelPorts[] = {20, 19};

void initializePIDs() {


    float driveKp = 0.2;
    float driveKi = 0.00000035;
    float driveKd = 0.0001;

    wheelsLeft  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, 20);
    wheelsRight = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, 20);
 printf("PID init ports: L = %.2f, %.2f | R = %.2f, %.2f\n", leftWheelPorts[0], leftWheelPorts[1], rightWheelPorts[0], rightWheelPorts[1]);
 printf("PID init: L = %.2f, %.2f | R = %.2f, %.2f\n", wheelsLeft.motorPorts[0], wheelsLeft.motorPorts[1], wheelsRight.motorPorts[0], wheelsRight.motorPorts[1]);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_DEGREES);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}


void moveIn(double left, double right) {
//  left *=0.5;
//  right *=0.5;
  PID_properties_t a[2] = {generateMovedPID(wheelsLeft, 360/(4*PI)*left), generateMovedPID(wheelsRight, 360/(4*PI)*right)};
  printf("start: %d  %d\n", a[0].error, a[1].error);
  printf("%d\n", atTarget(a[0]));
  bool flag = 0;
  //a[0].error = a[1].error;
  wheelsLeft = a[0];
  wheelsRight = a[1];

 while (!atTarget(a[0]) && !atTarget(a[1])) {
    a[0] = generateNextPID(a[0]);
    a[1] = generateNextPID(a[1]);
    //printf("Left: %d       Right: %d\n", a[1].error, a[0].error);
}
printf("Left: %d       Right: %d\n", a[1].error, a[0].error);
/*while (1) {
    if(!atTarget(a[0])){
      a[0] = generateNextPID(a[0]);
    }
    if(!atTarget(a[1])){
      a[1] = generateNextPID(a[1]);
    }
    if(atTarget(a[0]) & atTarget(a[1])){
      break;
    }
}*/
  wheelsLeft = a[0];
  wheelsRight = a[1];
  /*motor_move(MOTOR_FRONT_LEFT, 0);
  motor_move(MOTOR_FRONT_RIGHT, 0);
  motor_move(MOTOR_BACK_LEFT, 0);
  motor_move(MOTOR_BACK_RIGHT, 0);*/
}

void moveRaw(long raw) {
    wheelsLeft = generateMovedPID(wheelsLeft, raw);
    wheelsRight = generateMovedPID(wheelsRight, raw);

    // while (!atTarget(wheelsLeft) && !atTarget(wheelsRight)) {
    for (int i = 0; i < 100; i++) {
 printf("<LEFT> ");
        wheelsLeft = generateNextPID(wheelsLeft);
 printf("<RIGHT> ");
        wheelsRight = generateNextPID(wheelsRight);
 // printf("speed: %d, %d\n", wheelsLeft.speed, wheelsRight.speed);
 printf("\n");
    }
}
void moveRaw2(long raw) {

}
/*void moveIn(float inches) {
    moveRaw2(inches * MOTOR_COUNTS_PER_INCH);
}*/
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT,mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {

}

/*long leftDrivePos() {
	return (motor_get_position(leftPorts[0]) + motor_get_position(leftPorts[1])) / 2;
}
long rightDrivePos() {
	return (motor_get_position(rightPorts[0]) + motor_get_position(rightPorts[1])) / 2;
}
long drivePos() {
	return (leftDrivePos() + rightDrivePos()) / 2;
}*/
