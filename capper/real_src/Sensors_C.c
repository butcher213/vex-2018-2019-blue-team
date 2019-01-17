#include "../include/main_C.h"

PID_properties_t leftWheels, rightWheels;

void initializePIDs() {
    setupMotor(1, 1, E_MOTOR_GEARSET_18);
    setupMotor(2, 1, E_MOTOR_GEARSET_18);
    setupMotor(11, 0, E_MOTOR_GEARSET_18);
    setupMotor(12, 0, E_MOTOR_GEARSET_18);

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

void moveRaw(long raw) {
    leftWheels = generateMovedPID(leftWheels, raw);
    rightWheels = generateMovedPID(rightWheels, raw);

    // while (!atTarget(leftWheels) && !atTarget(rightWheels)) {
    for (int i = 0; i < 100; i++) {
 printf("<LEFT> ");
        leftWheels = generateNextPID(leftWheels);
 printf("<RIGHT> ");
        rightWheels = generateNextPID(rightWheels);
 // printf("speed: %d, %d\n", leftWheels.speed, rightWheels.speed);
 printf("\n");
    }
}
void moveIn(float inches) {
    moveRaw(inches * MOTOR_COUNTS_PER_INCH);
}
void moveMats(float mats) {
    moveIn(mats * INCHES_PER_MAT);
}

void rotateTo(float targetDeg) {

}
