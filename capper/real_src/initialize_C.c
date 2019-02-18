#include "../include/main_C.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
    printf("\n\nINIT START\n");

    initializePIDs();
    initializeMotors();

 #warning "Testing for moveIn() enabled"
    // moveIn(36);
    // moveMats(1);
    // rotateTo(ROBOT_ROTATION_TURN_LEFT * 180);
    // moveRaw(1000);

    // moveArmTo(POLE_PREPARE_HEIGHT);

    printf("\n\nINIT END\n");

    while (1) {
        printf("%f | %f | %f | %f    \n",
                motor_get_position(1),
                motor_get_position(2),
                motor_get_position(11),
                motor_get_position(12));

        delay(1000);
    }
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
