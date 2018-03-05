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
#pragma config(Motor,  port9,           bot4,          tmotorVex393_MC29, openLoop, reversed, driveRight)
#pragma config(Motor,  port10,          bot5,          tmotorVex393HighSpeed_HBridge, openLoop, reversed)

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

static const int LIFT_MIN = 2400;
static const int LIFT_MAX = 1000;

static const int CHAIN_MIN = 400;
static const int CHAIN_MAX = 3500;

static const int MOBILE_MIN = 0;
static const int MOBILE_MAX = 256;

static const int HOLDING_SPEED = 10;

static const int CONE_HEIGHT = 100;
static const int CONE_GRAB_HEIGHT = 3300;
static const int MAX_STACK_SIZE = 1200;

static const int MODE_AUTONOMOUS = 0;
static const int MODE_DRIVERTANK = 1;
static const int MODE_DRIVERVECTOR = 2;
static const int MODE_NUMBER = 3;

static const int DECELERATION_STEP_SIZE = 50;
static const int ACCELERATION_STEP_SIZE = 40;

static const int INTEGRAL_ACTIVATION = 0.05;

static const int CHECKS = 3;


void determineMotorValuesTank();
void determineMotorValuesVector();

void initializeRobot();
void centerRobot();
void stack();
void stackRailing();
void grabCone();
void resetIntegrals();


task liftcontrol();
task kill();

int checkMode(int prevMode);
void resetAllSensors();

void tank(int leftSpd, int rightSpd);
bool turnTo(int degree);
void stopAllMotors();
void stopLiftMotors();

void liftSpeed(int speed);
void chainSpeed(int speed);
void mobileLiftSpeed(int speed);
void grabSpeed(int speed);

bool liftPosition(int potPos);
bool chainPosition(int potPos);
bool mobilePosition(int potPos);

void driveLift();
void driveGrabber();
void driveMobileLift();
void driveChain();

//Creates ability to make acceleration curve for the robot.
int leftSpeed = 0, rightSpeed = 0;
float leftDiff = 0, rightDiff = 0;

bool initialized = false;
bool autonRun = false;

int coneStackHeight = MAX_STACK_SIZE;
int whiteVal;

bool activated = false;

float liftIntegral = 0, chainIntegral = 0;

int chainPosCount = 0;

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
	resetAllSensors();
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


	grabSpeed(HOLDING_SPEED * -4);

	while(!chainPosition(1850)){
		delay(50);
	}

	chainSpeed(0);

	while(!liftPosition(2000)){
		delay(50);
	}

	liftSpeed(0);

	tank(-127,-127);

	delay(1500);

	stopAllMotors();

	delay(500);

	grabSpeed(127);

	delay(1000);

	tank(127,127);

	delay(900);

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

	//Integers next to task definitions define priority
	//startTask(liftcontrol);
	//startTask(kill);

 while(true){
		determineMotorValuesTank();


		if(vexRT[Btn8UXmtr2] == 1){
			stack();
		} else if(vexRt[Btn8DXmtr2] == 1){
			grabCone();
		} else if(vexRT[Btn8RXmtr2] == 1){
			stackRailing();
		}


		delay(50);
	}

	/*while(true){
		stackRailing();
	}*/

}

task liftcontrol(){
	while(true){

		if(!activated){
			if(vexRT[Btn8UXmtr2] == 1){
				stack();
			} else if(vexRt[Btn8DXmtr2] == 1){
				grabCone();
			} else if(vexRT[Btn8RXmtr2] == 1){
				stackRailing();
			}


		}
		EndTimeSlice();

	}
}

task kill(){

	bool stopLift = false;

	bool stopped = true;

	while(true){

		if(vexRT[Btn7UXmtr2] == 1){
			stopLift = !stopLift;
			stopped = false;
		}

		if(stopLift){
			if(!stopped){
				stopTask(liftcontrol);
				stopped = true;
				stopLiftMotors();
			}
		} else if(!stopLift){
			if(!stopped){
				startTask(liftcontrol);
				stopped = true;
			}
		}

		EndTimeSlice();

	}
}

int checkMode(int prevMode){
	if(vexRT[Btn8L] == 1){
		if(prevMode - 1 == -1){
			return prevMode;
		}

		return --prevMode;
	} else if(vexRT[Btn8R] == 1){
		if(prevMode + 1 == MODE_NUMBER){
			return prevMode;
		}

		return ++prevMode;
	} else {
		return prevMode;
	}
}

void resetAllSensors(){
	SensorValue[lift] = 0;
	SensorValue[chain] = 0;
	SensorValue[mobilelift] = 0;

	SensorValue[deg] = 0;

	SensorValue[leftLine] = 0;
	SensorValue[rightLine] = 0;

	SensorValue[quadLeft] = 0;
	SensorValue[quadRight] = 0;
}

void tank(int leftSpd, int rightSpd){

	/*
	 * leftSpd = preferred speed, leftSpeed = actual speed.
	 */

	leftDiff = ((float)(leftSpd - leftSpeed)/(127));
	rightDiff = ((float)(rightSpd - rightSpeed)/(127));

	if(leftSpd < leftSpeed){
		leftSpeed += ((leftDiff) * DECELERATION_STEP_SIZE);
	} else if(leftSpd > leftSpeed){
		leftSpeed += ((leftDiff) * ACCELERATION_STEP_SIZE);
	}

	if(rightSpd < rightSpeed){
		rightSpeed += ((rightDiff) * DECELERATION_STEP_SIZE);
	} else if(rightSpd > rightSpeed){
		rightSpeed += ((rightDiff) * ACCELERATION_STEP_SIZE);
	}

	motor[bot0] = -leftSpeed;
	motor[bot1] = -leftSpeed;

	motor[bot4] = -rightSpeed;
	motor[bot5] = -rightSpeed;
}

void stopAllMotors(){
	motor[bot0] = 0;
	motor[bot1] = -0;

	motor[bot4] = -0;
	motor[bot5] = 0;

}

void stopLiftMotors(){
	motor[liftLeft] = 0;
	motor[liftRight] = -0;

	motor[mobileLeft] = -0;
	motor[mobileRight] = 0;

	motor[grab] = 0;

}

bool turnTo(int degree){

	float diff = (SensorValue[deg] - degree)/3600.0;

	bool command = true;

	if(diff < INTEGRAL_ACTIVATION){
		diff += INTEGRAL_ACTIVATION;
	} else if(diff < INTEGRAL_ACTIVATION / 2){
		command = false;
	}


	if(command){
		tank((int)(diff * -127),(int)(diff * 127));
	} else {
	 tank(0,0);
	}

	return !command;
}

void liftSpeed(int speed){
	motor[liftLeft] = speed;
	motor[liftRight] = speed;
}

void chainSpeed(int speed){
	motor[mobileLeft] = -speed;
	motor[mobileRight] = -speed;
}

void mobileLiftSpeed(int speed){
	motor[chainLeft] = speed;
}

void grabSpeed(int speed){
	motor[grab] = speed;
}

bool liftPosition(int potPos){
	float diff = ((SensorValue[lift] - potPos))/(float)(LIFT_MIN - LIFT_MAX);

	bool liftPosCommand = true;

	if(diff < INTEGRAL_ACTIVATION*2 && diff > 0.01){
		liftIntegral += diff;
	} else if(diff > INTEGRAL_ACTIVATION*-2 && diff < -0.01){
		liftIntegral -= diff;
	} else if(diff < 0.01 && diff > -0.01){
		liftPosCommand = false;
		liftIntegral = 0;
	} else {
		liftIntegral += diff/2.0;
	}

	if(liftPosCommand){
		liftSpeed((int)((diff + liftIntegral) * -127));
	} else {
		liftSpeed(0);
	}

	return !liftPosCommand;

}

bool chainPosition(int potPos){
	float diff = ((SensorValue[chain] - potPos))/(float)(CHAIN_MAX - CHAIN_MIN);

	bool chainPosCommand = true;

	if(diff < INTEGRAL_ACTIVATION && diff > 0){
		chainIntegral += 0.005;
	} else if(diff > INTEGRAL_ACTIVATION && diff < 0){
		chainIntegral -= 0.005;
	} else if(diff < 0.05 && diff > -0.05){
		if(chainPosCount == CHECKS){
			chainPosCommand = false;
		} else {
			chainPosCount++;
		}

		//chainPosCommand = true;

		chainIntegral = 0;
	} else {
		chainIntegral += diff;
	}

	if(chainPosCommand){
		chainSpeed((float)((diff + chainIntegral) * 45));
	} else {
		chainSpeed(0);

		chainPosCount = 0;

	}

	return !chainPosCommand;
}

bool mobilePosition(int potPos){
	float diff = (SensorValue[mobilelift] - potPos)/4096.0;

	bool mobileLiftPosCommand = true;

	if(diff < INTEGRAL_ACTIVATION){
		diff += 0.075;
	} else if(diff < 0.01){
		mobileLiftPosCommand = false;
	}

	if(mobileLiftPosCommand){
		mobileLiftSpeed(diff * 127);
	} else {
		mobileLiftSpeed(0);
	}

	return !mobileLiftPosCommand;

}

void driveLift(){
	int liftPower;

	/*if(vexRT[Btn6UXmtr2] == 1){
		liftPower = 127;
	} else if(vexRt[Btn6DXmtr2] == 1){
		liftPower = -127;
	} else {
		liftPower = 0;
	}*/

	liftPower = -vexRt[Ch2Xmtr2];


	liftSpeed(liftPower);
}

void driveChain(){
	int chainPower;

	/*if(vexRT[Btn5UXmtr2] == 1){
		chainPower = 127;
	} else if(vexRT[Btn5DXmtr2] == 1){
		chainPower = -127;
	} else {
		chainPower = HOLDING_SPEED;
	}*/

	chainPower = vexRt[Ch3Xmtr2];

	chainSpeed(chainPower);
}

void driveMobileLift(){
	int mobileLiftPower;

	if(vexRT[Btn7UXmtr2] == 1){
		mobileLiftPower = 127;
	} else if(vexRT[Btn7DXmtr2] == 1){
		mobileLiftPower = -127;
	} else {
		mobileLiftPower = HOLDING_SPEED;
	}

	mobileLiftSpeed(mobileLiftPower);
}

void driveGrabber(){
	int grabPower;

	if(vexRT[Btn5UXmtr2] == 1){
		grabPower = 127;
	} else if(vexRT[Btn5DXmtr2] == 1){
		grabPower = -127;
	} else {
		grabPower = -HOLDING_SPEED;
	}

	grabSpeed(grabPower);
}

void determineMotorValuesTank(){
	int	rSpd = -vexRT[Ch3];
	int lSpd = -vexRT[Ch2];

	driveLift();
	driveChain();
	driveMobileLift();
	driveGrabber();

	tank(rSpd, lSpd);
}


void determineMotorValuesVector(){

	int speed = -vexRT[Ch3];
	int turn = -vexRT[Ch1];

	driveLift();
	driveChain();
	driveMobileLift();
	driveGrabber();

	int lSpd = (int)((speed + turn)/(127 * 2)) * 127;
	int rSpd = (int)((speed - turn)/(127 * 2)) * 127;

	tank(rSpd, lSpd);
}

void initializeRobot(){
	while(!mobilePosition(MOBILE_MIN) && !liftPosition(LIFT_MIN) && !chainPosition(CHAIN_MIN))
		delay(50);

	initialized = true;
}

void centerRobot(){
	float diff = 1;

	while(diff > 0.005){
		diff = (SensorValue[leftLine] - SensorValue[rightLine])/4096.0;
		tank(diff * -127, diff * 127);

		delay(50);

	}

	whiteVal = (SensorValue[leftLine] + SensorValue[rightLine])/2;

	SensorValue[deg] = 450;

}


void stack(){


	activated = true;

	int potVal = 0;

	while(!chainPosition(800)){
		delay(50);
		if(vexRT[Btn7u] == 1){
			return;
		}
	}

	resetIntegrals();

	chainSpeed(HOLDING_SPEED * -1);


	/*while(!liftPosition(LIFT_MAX)){
		if(SensorValue[coneUlt] < 15){
			break;
		}

		delay(50);
	}*/


	grabSpeed(-127);


	while(!liftPosition(LIFT_MIN)){
		delay(50);
		if(vexRT[Btn7u] == 1){
			return;
		}
	}

	resetIntegrals();

	liftSpeed(20);

	while(!chainPosition(0)){
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}
	}

	chainSpeed(HOLDING_SPEED * -1);

	resetIntegrals();

	delay(100);

	grabSpeed(HOLDING_SPEED * -1);

	while(!chainPosition(500)){
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}
	}

	resetIntegrals();

	chainSpeed(HOLDING_SPEED * -1);

	while(!liftPosition(LIFT_MAX)){
		if(SensorValue[coneUlt] > 15){
			break;
		}

		if(vexRT[Btn7U] == 1){
			return;
		}

		delay(50);
	}

	resetIntegrals();

	liftSpeed(10);

	int potPos = 0;

	while(SensorValue[chain] <= 1750){
		chainSpeed(-45);
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}


	}

	resetIntegrals();

	chainSpeed(0);

	delay(250);

	potPos = SensorValue{lift];

	while(!liftPosition(potPos + 45)){
		delay(20);
	}

	grabSpeed(127);

	delay(1000);

	grabSpeed(0);


	activated = false;

}

void stackRailing(){

	activated = true;

	grabSpeed(-127);

	int potVal = 0;

		while(!chainPosition(500)){
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}

	}

	resetIntegrals();

	chainSpeed(HOLDING_SPEED * -1);

	while(!liftPosition(2000)){
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}

	}

	resetIntegrals();




	/*while(!liftPosition(LIFT_MAX)){
		if(SensorValue[coneUlt] < 15){
			break;
		}

		delay(50);
	}*/

	//Uncomment to fix
	//grabSpeed(-127);


	//Originally 2250
	int railingHeight = 2250;

	while(!liftPosition(railingHeight)){
		delay(50);

		if(vexRT[Btn7U] == 1){
			return;
		}

	}

	resetIntegrals();



	/*while(!chainPosition(0)){
		delay(50);
	}

	chainSpeed(HOLDING_SPEED * -1);*/


	delay(100);

	grabSpeed(HOLDING_SPEED * -1);

	int potPos = 0;

	while(!liftPosition(LIFT_MAX)){
		if(SensorValue[coneUlt] > 15){
			break;
		}

		if(vexRT[Btn7U] == 1){
			return;
		}

		delay(50);
	}

	resetIntegrals();

	/*while(SensorValue[lift] < potPos + 10){
		liftSpeed(15);
		delay(50);
	}*/

	liftSpeed(0);

	while(SensorValue[chain] <= 1750){
		chainSpeed(-45);
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}
	}

	resetIntegrals();

	chainSpeed(0);

	delay(250);


	potPos = SensorValue[lift];

	while(!liftPosition(potPos + 45)){
		delay(20);
	}

	grabSpeed(127);

	delay(1000);

	grabSpeed(0);

	activated = false;

	//EndTimeSlice();
}


void grabCone(){

	activated = true;

	int potVal = 0;

	while(!chainPosition(800)){
		delay(50);
		if(vexRT[Btn7U] == 1){
			return;
		}
	}

	resetIntegrals();

	chainSpeed(HOLDING_SPEED * -1);


	/*while(SensorValue[coneUlt] < 15){
		potVal = SensorValue[lift];
		liftSpeed(-63);

		delay(50);
	}*/

	//liftSpeed(0);

	grabSpeed(-127);


	while(!liftPosition(LIFT_MIN)){
		delay(50);

		if(vexRT[Btn7U] == 1){
			return;
		}

	}

	resetIntegrals();

	liftSpeed(20);

	activated = false;

	//EndTimeSlice();

}

void resetIntegrals(){
	liftIntegral = 0;
	chainIntegral = 0;

}
