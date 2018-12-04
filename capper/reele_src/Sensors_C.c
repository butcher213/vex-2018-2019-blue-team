// #include "../include/Sensors_C.h"
// #include "../../include/PID.h"
#include "../include/main_C.h"
// #ifdef _PID_H_
// #warning "SENSORS: PID DEFINED"
// #endif

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

void moveRaw(long raw) {
    leftWheels = generateMovedPID(leftWheels, raw);
    rightWheels = generateMovedPID(rightWheels, raw);

    while (!atTarget(leftWheels) && !atTarget(rightWheels)) {
        leftWheels = generateNextPID(leftWheels);
        rightWheels = generateNextPID(rightWheels);
printf("l:%.2f | r:%.2f", leftWheels.error, rightWheels.error);
    }
}
void moveIn(float inches) {
    moveRaw(MOTOR_COUNTS_PER_INCH);
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
