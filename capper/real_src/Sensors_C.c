#include "../include/main_C.h"

PID_properties_t wheelsLeft, wheelsRight;
int leftWheelPorts[] = {11, 12};
int rightWheelPorts[] = {1, 2};

void initializePIDs() {
    setupMotor(DRIVE_RIGHT_FRONT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_RIGHT_REAR_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_FRONT_PORT, 0, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_REAR_PORT, 0, E_MOTOR_GEARSET_18);


    float driveKp = 0.2;
    float driveKi = 0.0000018; // orig = 0.00000035
    float driveKd = 0.0001;

    wheelsLeft  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, 20);
    wheelsRight = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, 20);
 printf("PID init ports: L = %d,%d | R = %d, %d\n", leftWheelPorts[0], leftWheelPorts[1], rightWheelPorts[0], rightWheelPorts[1]);
 printf("PID init: L = %d,%d | R = %d, %d\n", wheelsLeft.motorPorts[0], wheelsLeft.motorPorts[1], wheelsRight.motorPorts[0], wheelsRight.motorPorts[1]);
}

void initializeMotors() {
    setupMotor(ARM_LEFT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(ARM_RIGHT_PORT, 0, E_MOTOR_GEARSET_18);

    setupMotor(CLAW_ROTATE_PORT, 0, E_MOTOR_GEARSET_18);
    setupMotor(CLAW_PORT, 0, E_MOTOR_GEARSET_18);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_DEGREES);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}

void moveDrivePID() {
    while (!atTarget(wheelsLeft) && !atTarget(wheelsRight)) {
    // for (int i = 0; i < 100; i++) {
 // printf("<LEFT> ");
        wheelsLeft = generateNextPID(wheelsLeft);
 // printf("<RIGHT> ");
        wheelsRight = generateNextPID(wheelsRight);
 // printf("error: %lld, %lld\n", wheelsLeft.error, wheelsRight.error);
 // printf("speed: %lld, %lld\n", wheelsLeft.previousError, wheelsRight.previousError);
 printf("encoder: %.2f, %.2f\n", motor_get_position(1), motor_get_position(11));
 printf("\n");
    }
}

void moveRaw(long raw) {
    wheelsLeft = generateMovedPID(wheelsLeft, raw);
    wheelsRight = generateMovedPID(wheelsRight, raw);

    moveDrivePID();
}
void moveIn(float inches) {
    moveRaw(inches * MOTOR_DEGREES_PER_INCH);
}
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {
    long targetRaw = targetDeg * ROBOT_ROTATION_COUNTS_PER_DEGREE;

    wheelsLeft = generateMovedPID(wheelsLeft, targetRaw);
    wheelsRight = generateMovedPID(wheelsRight, -targetRaw);

    moveDrivePID();
}

long leftDrivePos() {
	return (motor_get_position(wheelsLeft.motorPorts[0]) + motor_get_position(wheelsLeft.motorPorts[1])) / 2;
}
long rightDrivePos() {
	return (motor_get_position(wheelsRight.motorPorts[0]) + motor_get_position(wheelsRight.motorPorts[1])) / 2;
}
long drivePos() {
	return (leftDrivePos() + rightDrivePos()) / 2;
}
