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
	motor_set_gearing(1, E_MOTOR_GEARSET_18);
	motor_set_reversed(1, 0);
	motor_set_encoder_units(1, E_MOTOR_ENCODER_DEGREES);
	motor_set_gearing(2, E_MOTOR_GEARSET_18);
	motor_set_reversed(2, 0);
	motor_set_encoder_units(2, E_MOTOR_ENCODER_DEGREES);
printf("setup motor\n");

	int leftPorts[] = {1};
	PID_properties_t left = createPID(.5, .000009, .009, leftPorts, 1, 40);
printf("setup PID_properties_t\n");

	left.target = 1000;
printf("Starting loop\n");
	while (left.error > 0) {
		left = updatePID(left);
printf("err: %5f | itgrl: %5f\n",
		left.error,
		left.integral);
	}

printf("%-7.2f", motor_get_position(left.motorPorts[0]));
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
