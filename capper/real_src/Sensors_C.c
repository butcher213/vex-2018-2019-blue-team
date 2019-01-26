#include "../include/main_C.h"

PID_properties_t wheelsLeft, wheelsRight;
int leftWheelPorts[] = {11, 12};
int rightWheelPorts[] = {1, 2};

void initializePIDs() {
    setupMotor(1, 1, E_MOTOR_GEARSET_18);
    setupMotor(2, 1, E_MOTOR_GEARSET_18);
    setupMotor(11, 0, E_MOTOR_GEARSET_18);
    setupMotor(12, 0, E_MOTOR_GEARSET_18);


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
	motor_set_encoder_units(port, E_MOTOR_ENCODER_COUNTS);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
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
	
	while (drivePos()
}
void moveIn(float inches) {
    moveRaw2(inches * MOTOR_COUNTS_PER_INCH);
}
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {

}

long leftDrivePos() {
	return (motor_get_position(leftPorts[0]) + motor_get_position(leftPorts[1])) / 2;
}
long rightDrivePos() {
	return (motor_get_position(rightPorts[0]) + motor_get_position(rightPorts[1])) / 2;
}
long drivePos() {
	return (leftDrivePos() + rightDrivePos()) / 2;
}