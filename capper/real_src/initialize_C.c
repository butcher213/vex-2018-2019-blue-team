#include "../include/main_C.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
	setupMotor(10, 0, E_MOTOR_GEARSET_18);
	setupMotor(9, 0, E_MOTOR_GEARSET_18);
	setupMotor(20, 1, E_MOTOR_GEARSET_18);
	setupMotor(19, 1, E_MOTOR_GEARSET_18);
    printf("\n\nINIT START\n");
    initializePIDs();
    initializeMotors();
	adi_port_set_config(ARM_POTENTIOMETER_PORT, E_ADI_ANALOG_IN);

 #warning "Testing for moveIn() enabled"
    // moveIn(36);
    // moveMats(1);
    // rotateTo(ROBOT_ROTATION_TURN_RIGHT * 180);
	

	// preload_shooter();
	// get_pole_side_blue_cap();
	// give_pole_side_blue_cap_balls_to_shooter();
	// place_first_cap_on_pole();
	// autonomous();
	
	moveArmTo(CAP_FLIP_HEIGHT);
	delay(1000);
	
    printf("\n\nINIT END\n");

    while (1) {
		// moveArmTo(CAP_HEIGHT);
		// delay(1000);
		// moveArmTo(CAP_FLIP_HEIGHT);
		// delay(1000);
		// moveArmTo(POLE_SMALL_HEIGHT);
		// delay(1000);
		// moveArmTo(POLE_BIG_HEIGHT);
		// delay(1000);
		// moveArmTo(POLE_PREPARE_HEIGHT);
		// delay(1000);
		
        printf("%ld | %lf | %lf | %lf    \n",
                adi_analog_read(ARM_POTENTIOMETER_PORT),
                motor_get_position(15),
                motor_get_position(5),
                motor_get_position(15));
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
