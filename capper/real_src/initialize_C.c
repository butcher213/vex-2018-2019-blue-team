#include "../include/main_C.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
  setupMotor(10, 0, E_MOTOR_GEARSET_18);
  setupMotor(20, 1, E_MOTOR_GEARSET_18);
  setupMotor(9, 1, E_MOTOR_GEARSET_18);
  setupMotor(19, 0, E_MOTOR_GEARSET_18);
    initializePIDs();

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
