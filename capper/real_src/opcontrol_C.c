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
    int _armSpeed = 127 * ARM_UP * controller_get_digital(ARM_CONTROLLER, ARM_UP_BTN);
    _armSpeed += 127 * ARM_DOWN * controller_get_digital(ARM_CONTROLLER, ARM_DN_BTN);

    printf("%d\n", _armSpeed);
    armSpeed(_armSpeed);
}

void driveControl() {
    int leftDriveSpeed = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STICK);
    int rightDriveSpeed = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STICK);

    leftDrive(leftDriveSpeed);
    rightDrive(rightDriveSpeed);
}

void clawControl() {
    int _clawSpeed = 127 * CLAW_OPEN * controller_get_digital(CLAW_CONTROLLER, CLAW_OPEN_BTN);
    _clawSpeed += 127 * CLAW_CLOSE * controller_get_digital(CLAW_CONTROLLER, CLAW_CLOSE_BTN);

    int clawRotateSpeed = 127 * CLAW_CW * controller_get_digital(CLAW_CONTROLLER, CLAW_CW_BTN);
    clawRotateSpeed += 127 * CLAW_CCW * controller_get_digital(CLAW_CONTROLLER, CLAW_CCW_BTN);

    clawSpeed(_clawSpeed);
    clawRotate(clawRotateSpeed);
}

void opcontrol() {
    while (1) {
        armControl();
        driveControl();
        clawControl();
    }
}
