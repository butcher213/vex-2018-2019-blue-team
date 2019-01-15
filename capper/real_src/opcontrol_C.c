#include "../include/main_C.h"
#include "../include/Motors_C.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void armControl() {
    int armSpeed = controller_get_digital(ARM_CONTROLLER, ARM_UP_BTN);

    armSpeed(armSpeed);
}

void driveControl() {
    int leftDriveSpeed = controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STICK);
    int rightDriveSpeed = controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STICK);

    leftDrive(leftDriveSpeed);
    rightDrive(rightDriveSpeed);
}

void clawControl() {
    int clawSpeed = controller_get_digital(CLAW_CONTROLLER, CLAW_BTN);
    int clawRotateSpeed = controller_get_digital(CLAW_CONTROLLER, CLAW_ROTATE_BTN);

    clawSpeed(clawSpeed);
    clawRotate(clawRotateSpeed);
}

void opcontrol() {
    armControl();
    driveControl();
    clawControl();
}
