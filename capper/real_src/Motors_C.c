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

void clawRotate(int speed) {
    motor_move(CLAW_ROTATE_PORT, speed);
}

void clawRotatePos(int pos, int speed) {
    int lastPosition;
    int stallTicks = 0;

    clawRotate(speed);

    while ((lastPosition = motor_get_position(CLAW_ROTATE_PORT)) != pos && stallTicks < 100) {
        delay(1);
        if (lastPosition == motor_get_position(CLAW_ROTATE_PORT))
            stallTicks++;
        else
            stallTicks = 0;
    }

    clawRotate(0);
}

// void clawSpeed(int speed) {
//     motor_move(CLAW_PORT, speed);
// }

void moveArmTo(int pos) {
    if (pos < armPosAvg()) {
        armSpeed(ARM_UP);

        while (armPosAvg() < pos)
            /* do  nothing */;

        armSpeed(0);
    }
    else {
        armSpeed(ARM_DOWN);

        while (armPosAvg() > pos)
            /* do  nothing */;

        armSpeed(0);
    }
}

void claw180Rotate() {
    switch (capFlipState) {
        case 0: // Must rotate CW
            clawRotatePos(-90, CLAW_CW);
            capFlipState = 1;
        break;
        case 1: // Must rotate CCW
            clawRotatePos(90, CLAW_CCW);
            capFlipState = 0;
        break;
        default: // default DFA to state 0
            capFlipState = 0;
            claw180Rotate();
    }
}
