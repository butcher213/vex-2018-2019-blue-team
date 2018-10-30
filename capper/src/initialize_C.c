#include "../include/main_C.h"
#include "../../include/PID.h"

void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	motor_set_gearing(1, E_MOTOR_GEARSET_36);
	motor_set_reversed(1, 0);
	motor_set_encoder_units(1, E_MOTOR_ENCODER_DEGREES);
	motor_set_gearing(2, E_MOTOR_GEARSET_36);
	motor_set_reversed(2, 1);
	motor_set_encoder_units(2, E_MOTOR_ENCODER_DEGREES);
printf("setup motor\n");

	int leftPorts[] = {1};
    int rightPorts[] = {2};
	double 	Kp = 0.5,
			Ki = 0.00005,
			Kd = 0.1;
	PID_properties_t left = createPID(Kp, Ki, Kd, leftPorts, 1, 40);
    PID_properties_t right = createPID(Kp, Ki, Kd, rightPorts, 1, 40);
printf("setup PID_properties_t\n");

    left = generateMovedPID(left, 360);
    right = generateMovedPID(right, 360);
    while (abs(left.error) || abs(right.error) > 0) {
        left = generateNextPID(left);
        right = generateNextPID(right);
printf("left   err: %5.2f | itgrl: %5d | drv: %5d\n",
				left.error,
				left.integral,
                left.derivative);
printf("right  err: %5.2f | itgrl: %5d | drv: %5d\n",
				right.error,
				right.integral,
                right.derivative);
    }

    // PID_properties_t *drive = rotateDrive(left, right, 360);
    // left = drive[0];
    // right = drive[1];
/*
    left = moveTarget(left, 360);
printf("Starting loop\n");
	do {
printf("tick\n");
		left = updatePID(left);
printf("err: %5.2f | itgrl: %5d | drv: %5d\n",
				left.error,
				left.integral,
                left.derivative);
    } while (1 || (int) abs(left.error) > 0);
	motor_move(1, 0);
*/
printf(">> %-7.2f\n", left.error);
printf("END");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
