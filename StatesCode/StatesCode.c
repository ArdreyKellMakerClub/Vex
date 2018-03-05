#pragma config(Sensor, in1,    deg,            sensorGyro)
#pragma config(Sensor, in2,    chain,          sensorPotentiometer)
#pragma config(Sensor, in3,    lift,           sensorPotentiometer)
#pragma config(Sensor, in4,    leftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    rightLine,      sensorLineFollower)
#pragma config(Sensor, in6,    mobilelift,     sensorPotentiometer)
#pragma config(Sensor, dgtl1,  quadLeft,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  quadRight,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  coneUlt,        sensorSONAR_cm)
#pragma config(Motor,  port1,           bot0,          tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           bot1,          tmotorVex393_MC29, openLoop, driveLeft, encoderPort, dgtl1)
#pragma config(Motor,  port3,           mobileLeft,    tmotorVex393_MC29, openLoop, reversed, driveLeft)
#pragma config(Motor,  port4,           liftLeft,      tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port5,           chainLeft,     tmotorVex393_MC29, openLoop, driveLeft)
#pragma config(Motor,  port6,           grab,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           liftRight,     tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port8,           mobileRight,   tmotorVex393_MC29, openLoop, driveRight, encoderPort, dgtl3)
#pragma config(Motor,  port9,           bot4,          tmotorVex393_MC29, openLoop, driveRight)
#pragma config(Motor,  port10,          bot5,          tmotorVex393HighSpeed_HBridge, openLoop)

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*
 * All controller buttons
 */
#define VEX_LEFT_DRIVE Ch2
#define VEX_RIGHT_DRIVE Ch3
#define JOY_LIFT Ch3xmtr2
#define JOY_CHAIN Ch2xmtr2
#define BTN_MAN_MOGO_IN Btn7Uxmtr2
#define BTN_MAN_MOGO_OUT Btn7Dxmtr2
#define BTN_GRAB Btn5Uxmtr2
#define BTN_RELEASE Btn5Dxmtr2
#define BTN_STACK Btn8Uxmtr2
#define BTN_UNSTACK Btn8Dxmtr2
#define BTN_GRAB_STACK Btn8Lxmtr2
#define BTN_RAILING_STACK Btn8Rxmtr2
#define BTN_RELEASE_MOGO Btn8U
#define KILL Btn7U

#define LIFT_MIN 2400;
#define LIFT_MAX 1000;
#define CHAIN_MIN 400;
#define CHAIN_MAX 3500;
#define HOLDING_SPEED 10;
#define DECELERATION_STEP_SIZE 50;
#define ACCELERATION_STEP_SIZE 40;
#define INTEGRAL_ACTIVATION 0.05;
#define CHECKS 3;

task liftcontrol();
task kill();

void stopDriveMotors();
void stopLiftMotors();

// Used to create the acceleration curve.
int leftSpeed = 0;
int rightSpeed = 0;
float leftDiff = 0;
float rightDiff = 0;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

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

task autonomous()
{
  // ..........................................................................
  // Insert user code here.
  // ..........................................................................

  // Remove this function call once you have "real" code.
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop

	startTask(kill);
	startTask(liftcontrol);

  while (true)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.


    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Remove this function call once you have "real" code.

  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Lift Control Task                            */
/*                                                                           */
/*     This task controls all of the lift functions including stacking and   */
/*										           manual control                              */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


task liftcontrol(){



}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Kill Task                                    */
/*                                                                           */
/*              Kills the program whenever any error occurs.                 */
/*										                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
task kill(){

	// Runs a background task by putting code in a while loop
	while(true){

		// Checks if the kill button has been pressed
		if(vexRT[KILL] == 1){

			// Stops the liftcontrol task, preventing the lift from moving
			stopTask(liftcontrol);
			stopLiftMotors();

			// Stops the thread for 0.5 seconds before giving the user control again
			delay(500);
			startTask(liftcontrol);
		}
	}

}

/*
 * Stops all of the lift motors
 */
void stopLiftMotors(){
	motor[liftLeft] = 0;
	motor[liftRight] = 0;
}

/*
 * Stops all of the drive motors
 */
void stopDriveMotors(){
	motor[bot0] = 0;
	motor[bot1] = 0;
	motor[bot4] = 0;
	motor[bot5] = 0;
}

void tankBasic(int leftSpd, int rightSpd){
	motor[bot0] = leftSpd;
	motor[bot1] = leftSpd;
	motor[bot4] = rightSpd;
	motor[bot5] = rightSpd;
}

void tankAcceleration(int prefLeftSpd, int prefRightSpd){

	/*
	 * leftSpd = preferred speed, leftSpeed = actual speed.
	 */


	/*
	 * Calculates the difference between the leftSpeed the preferred Speed, then creates a ratio
	 * according to the values.
	 */
	leftDiff = (float)(prefLeftSpd - leftSpeed)/(127);
	rightDiff = (float)(prefRightSpd - rightSpeed)/(127);

	/*
	 * Determines whether the ACCELERATION_STEP_SIZE or DECELERATION_STEP_SIZE should be used
	 * Adds the speed to the corresponding variable, then sets the speed.
	 */
	if(prefLeftSpd < leftSpeed){
		leftSpeed += (leftDiff * DECELERATION_STEP_SIZE);
	} else if(prefLeftSpd > leftSpeed){
		leftSpeed += (int)(leftDiff * ACCELERATION_STEP_SIZE);
	}

	if(prefRightSpd < rightSpeed){
		rightSpeed += (int)(rightDiff * DECELERATION_STEP_SIZE);
	} else if(prefRightSpd > rightSpeed){
		rightSpeed += (int)(rightDiff * ACCELERATION_STEP_SIZE);
	}

	motor[bot0] = leftSpeed;
	motor[bot1] = leftSpeed;

	motor[bot4] = rightSpeed;
	motor[bot5] = rightSpeed;

}
