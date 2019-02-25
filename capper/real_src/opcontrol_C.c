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
    int leftY  = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STICK);
    int rightY = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STICK);
    int leftX  = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_LEFT_STRAFE_STICK);
    int rightX = DRIVE_FORWARD * controller_get_analog(DRIVE_CONTROLLER, DRIVE_RIGHT_STRAFE_STICK);

//---- TANK DRIVE
    // int driveScaledL = (leftY * leftY * leftY) / (127 * 127);
    // int driveScaledR = (rightY * rightY * rightY) / (127 * 127);
    // leftDrive(driveScaledL);
    // rightDrive(driveScaledR);

//---- ARCADE DRIVE [R] (RIGHT DRIVE, LEFT TURN)
    // int turningScaled = (leftX * leftX * leftX) / (127 * 127);
    // int driveScaled = (rightY * rightY * rightY) / (127 * 127);
    // leftDrive(driveScaled + turningScaled);
    // rightDrive(driveScaled - turningScaled);

//---- ARCADE DRIVE [L] (LEFT DRIVE, RIGHT TURN)
    int turningScaled = (rightX * rightX * rightX) / (127 * 127);
    int driveScaled = (leftY * leftY * leftY) / (127 * 127);
    leftDrive(driveScaled - turningScaled);
    rightDrive(driveScaled + turningScaled);
}

void clawControl() {
    // int _clawSpeed = 127 * CLAW_OPEN * controller_get_digital(CLAW_CONTROLLER, CLAW_OPEN_BTN);
    // _clawSpeed += 127 * CLAW_CLOSE * controller_get_digital(CLAW_CONTROLLER, CLAW_CLOSE_BTN);
    //
    // int clawRotateSpeed = 127 * CLAW_CW * controller_get_digital(CLAW_CONTROLLER, CLAW_CW_BTN);
    // clawRotateSpeed += 127 * CLAW_CCW * controller_get_digital(CLAW_CONTROLLER, CLAW_CCW_BTN);
    //
    // clawSpeed(_clawSpeed);
    // clawRotate(clawRotateSpeed);

    if(controller_get_digital(CLAW_CONTROLLER, CLAW_ROTATE_BTN)) {
        claw180Rotate();
    }
}

void opcontrol() {
    moveIn(18,18);
    while (1) {
        armControl();
        driveControl();
        clawControl();
    }
}
