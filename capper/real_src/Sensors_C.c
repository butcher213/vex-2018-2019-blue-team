#include "../include/main_C.h"

PID_properties_t wheelsLeft, wheelsRight;
int leftWheelPorts[] = {11, 12};
int rightWheelPorts[] = {1, 2};

void initializePIDs() {
    setupMotor(DRIVE_RIGHT_FRONT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_RIGHT_REAR_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_FRONT_PORT, 0, E_MOTOR_GEARSET_18);
    setupMotor(DRIVE_LEFT_REAR_PORT, 0, E_MOTOR_GEARSET_18);


    float driveKp = 0.35;     // orig = .2
    float driveKi = 0.000007; // orig = .0000018
    float driveKd = 0.003;   // orig = .0001
    long startStopping = 100;

    wheelsLeft  = createPID(driveKp, driveKi, driveKd, leftWheelPorts,  2, startStopping);
    wheelsRight = createPID(driveKp, driveKi, driveKd, rightWheelPorts, 2, startStopping);
 printf("PID init ports: L = %d,%d | R = %d, %d\n", leftWheelPorts[0], leftWheelPorts[1], rightWheelPorts[0], rightWheelPorts[1]);
 printf("PID init: L = %d,%d | R = %d, %d\n", wheelsLeft.motorPorts[0], wheelsLeft.motorPorts[1], wheelsRight.motorPorts[0], wheelsRight.motorPorts[1]);
}

void initializeMotors() {
    setupMotor(ARM_LEFT_PORT, 1, E_MOTOR_GEARSET_18);
    setupMotor(ARM_RIGHT_PORT, 0, E_MOTOR_GEARSET_18);

    setupMotor(CLAW_ROTATE_PORT, 0, E_MOTOR_GEARSET_18);
    // setupMotor(CLAW_PORT, 0, E_MOTOR_GEARSET_18);
}

void setupMotor(int port, int reversed, int gearset) {
	motor_set_gearing(port, gearset);
	motor_set_reversed(port, reversed);
	motor_set_encoder_units(port, E_MOTOR_ENCODER_DEGREES);
    motor_set_brake_mode(port, E_MOTOR_BRAKE_COAST);
}

void moveDrivePID() {
    int i = 0;
    while (i < 2) {
        i = 0;
//  printf("<LEFT> ");
        if (!atTarget(wheelsLeft))
            wheelsLeft = generateNextPID(wheelsLeft);
        else
            i++;
//  printf("<RIGHT> ");
        if (!atTarget(wheelsRight))
            wheelsRight = generateNextPID(wheelsRight);
        else
            i++;
//  printf("error: %lld, %lld\n", wheelsLeft.error, wheelsRight.error);
//  printf("speed: %lld, %lld\n", wheelsLeft.previousError, wheelsRight.previousError);
//  printf("encoder: %.2f, %.2f\n", motor_get_position(1), motor_get_position(11));
//  printf("targ: %.2f, %.2f\n", wheelsLeft.target, wheelsRight.target);
// printf("\n");
    }

    motor_move(1, 0);
    motor_move(2, 0);
    motor_move(11, 0);
    motor_move(12, 0);
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
 printf("%ld | %f, %lf\n", targetRaw, targetDeg, ROBOT_ROTATION_COUNTS_PER_DEGREE);
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

int armPosAvg() {
    return (motor_get_position(ARM_LEFT_PORT) + motor_get_position(ARM_RIGHT_PORT))/2;
}
