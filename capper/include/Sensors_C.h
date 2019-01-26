#ifndef _SENSORS_H_
#define _SENSORS_H_

#define PI 3.1415
#define WHEEL_DIAMETER ((4 + 4.25)/2)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define MOTOR_COUNT_PER_REVOLUTION 4554752
#define MOTOR_COUNTS_PER_INCH ((double) MOTOR_COUNT_PER_REVOLUTION / WHEEL_CIRCUMFERENCE)
#define Pole_Hight_Small 23.0
#define Pole_Hight_Large 34.0
#define WALL_TO_WALL_INCHES 140.5
#define WALL_TO_WALL_MATS 6
// ~23.42 inches per mat
#define INCHES_PER_MAT (WALL_TO_WALL_INCHES / WALL_TO_WALL_MATS)


/* Function:		initializePID
 * Purpose:			Initializes the PID objects for the robot.
 * Argument:		N/A
 * Returns:			N/A
 */
void initializePIDs();

/* Function:        setupMotor
 * Purpose:			Initializes the motor with raw encoder counts.
 * Argument:		port = smart port to setup
                    reversed = 1 to reverse motor
                    gearset = the gearset of the motor
 * Returns:			N/A
 */
void setupMotor(int port, int reversed, int gearset);

/* Function:		moveRaw
 * Purpose:			move the robot the specified number of raw counts
 * Argument:		raw = number of raw encoder counts to move
 * Returns:			N/A
 */
void moveRaw(long raw);
void moveRaw2(long raw);

/* Function:		moveIn
 * Purpose:			moves the robot a specified amount of inches
 * Argument:		inches = amount of inches to move
 * Returns:			N/A
 */
void moveIn(float inches);

/* Function:		moveMats
 * Purpose:			moves the robot a specified amount of mats
 * Argument:		mats = amount of mats to move
 * Returns:			N/A
 */
void moveMats(float mats);

/* Function:		rotateTo
 * Purpose:			rotates the robot to the specified degree
 * Argument:		deg = degree to move to
 * Return:			N/A
 */
void rotateTo(float targetDeg);

long leftDrivePos();
long rightDrivePos();
long drivePos();


#endif // _SENSORS_H_
