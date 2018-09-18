#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    Gyro,           sensorGyro)
#pragma config(Sensor, in2,    ArmPotentiometer, sensorPotentiometer)
#pragma config(Sensor, dgtl1,  GoalLiftBump,   sensorTouch)
#pragma config(Sensor, dgtl2,  ArmEncoder,     sensorNone)
#pragma config(Sensor, dgtl3,  redOrBlue,      sensorTouch)
#pragma config(Sensor, I2C_1,  EncoderLeft,    sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  EncoderStrafe,  sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  EncoderRight,   sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           WheelsLeft,    tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,            ,             tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port4,           WheelStrafe,   tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port5,           GoalArm,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           ConeArm,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           ConeClaw,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           GoalPusher,    tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           WheelsRight,   tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

#include "Tele.h"
#include "Auton.h"
#include "Init.h"

//===================================================================================

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton() {
  doPreAuton();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous(){
	doAuton();
}


/*-----------------0-----------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
task usercontrol() {
	//while(true) {
	//	doTeleop();
	//	}
doAuton();
}
