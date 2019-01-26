#include "../include/main_C.h"
#include "../include/Motors_C.h"

void armSpeed(int speed) {
    motor_move(ARM_LEFT_PORT, speed);
    motor_move(ARM_RIGHT_PORT, speed);
}

void leftDrive(int speed) {
    motor_move(DRIVE_LEFT_FRONT_PORT, speed);
    motor_move(DRIVE_LEFT_REAR_PORT, speed);
}

void rightDrive(int speed) {
    motor_move(DRIVE_RIGHT_FRONT_PORT, speed);
    motor_move(DRIVE_RIGHT_REAR_PORT, speed);
}

void clawRotate(int direction) {
    motor_move(CLAW_ROTATE_PORT, direction);
}

void clawSpeed(int speed) {
    motor_move(CLAW_PORT, speed);
}

void t() {
    armSpeed(10);
}
