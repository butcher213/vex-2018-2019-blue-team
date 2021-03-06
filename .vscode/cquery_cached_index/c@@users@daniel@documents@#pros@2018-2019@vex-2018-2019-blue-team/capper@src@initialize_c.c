// #include "../../include/PID.h"
// #include "../include/Sensors_C.h"
#include "../include/main_C.h"
#ifdef _PID_H_
#warning "INITIALIZE: PID DEFINED"
#endif

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
**/
void initialize() {
    initializePIDs();

    setupMotor(1, 1, E_MOTOR_GEARSET_18);
    setupMotor(2, 1, E_MOTOR_GEARSET_18);
    setupMotor(11, 0, E_MOTOR_GEARSET_18);
    setupMotor(12, 0, E_MOTOR_GEARSET_18);

    motor_move(1, 127);
    while(1);
	// int leftPorts[] = {11, 12};
	// int rightPorts[] = {1, 2};
	// PID_properties_t left, right;
	// float Kp = .2;
	// float Ki = .00000035;
	// float Kd =  0.0001;
	// left = createPID(Kp, Ki, Kd, leftPorts, 2, 20);
    // right = createPID(Kp, Ki, Kd, rightPorts, 2, 20);
	// left = generateMovedPID(left, 24 * (360 / (PI * 3.6)));
	// right = generateMovedPID(right, 24 * (360 / (PI * 3.6)));
	// while(1) { //&& !atTarget(right)) {
	// 	left = generateNextPID(left);
	// 	right = generateNextPID(right);
	// 	printf("%7.2f\n", left.error);
	// }
	/*
    motor_move(1, 0);
	motor_move(2, 0);
	motor_move(11, 0);
	motor_move(12, 0);
    int a = 1;
    while (a) {
        motor_move(1, 60);
        motor_move(2, 60);
        motor_move(11, 60);
        motor_move(12, 60);
    }

printf("button: %d\n", adi_port_set_config('A', E_ADI_DIGITAL_IN));
printf("setup motor\n");

	int leftPorts[] = {11, 12};
    int rightPorts[] = {1, 2};
	double 	Kp = .8,
			Ki = 0,
			Kd = .55;
    int dist = 20 * (360 / 12.566368);

    int drivePorts[] = {1, 2, 11, 12};
    findKpid_Ziegler(drivePorts, 4, 40, dist);

    // PID_properties_t left, right;
    // for (; !adi_digital_read('A'); Kd += .1) {
    	// left = createPID(Kp, Ki, Kd, leftPorts, 1, 40);
        // right = createPID(Kp, Ki, Kd, rightPorts, 1, 40);
printf("setup PID_properties_t\n");
        // PID_array_t pids = generateRotatedDrive(left, right, dist * 90 / (3.1415 * 15));
        // left = pids[0];
        // right = pids[1];

        // left = generateMovedPID(left, dist);
        // right = generateMovedPID(right, dist);

        // int i;
        // for (; i < 1000 && (!atTarget(left) || !atTarget(right)); i++) {
<<<<<<< HEAD
        for (i = 0; (!atTarget(left) || !atTarget(right)) && i < 200; i++) {
            left = generateNextPID(left);
            right = generateNextPID(right);
printf("%7.2f | %7.2f\n", left.integral, right.integral);*/
// printf("%7.5f | %7.5f | %7.5f\n", Kp, Ki, Kd);
// printf("left   er: %5.2f : %5.2f | i: %12d | d: %12d | t: %5d : %5d\n",
// 		left.error,
//         left.previousError,
// 		left.integral,
//         left.derivative,
//         left.target,
//         motor_get_position(11));
// printf("right  er: %5.2f : %5.2f | i: %12d | d: %12d | t: %5d : %5d\n",
// 		right.error,
//         right.previousError,
// 		right.integral,
//         right.derivative,
//         right.target,
//         motor_get_position(1));
  //      }
    // }
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
// printf(">> %-7.2f | %-7.2f\n", left.error, right.error);
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
