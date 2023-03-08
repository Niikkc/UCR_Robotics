#include "main.h"
#include <iostream>
#include <string.h>

using namespace std;

// MOTOR DECLARATIONS

const pros::Motor leftMotorFront(13);
const pros::Motor leftMotorBack(15);

const pros::Motor rightMotorFront(16, true);
const pros::Motor rightMotorBack(11, true);

const pros::Motor catapult(14);
const pros::Motor intake(6);
const pros::Motor roller(1);
const pros::Motor expansion(10, true);


// == CONSTANTS == \\ 

const float wheelDiameter = 4; 
const float wheelGearRatio = 1;												// Used if we're using gears
const float wheelCircumference = wheelDiameter * 3.141592653589793238462;
const double inchesPerDegree = wheelCircumference / 900;					// Inches moved per motor degree (wheels)
const float slightFix = 0.96; 												// multiplier to account for scuffed motors
const float autonPowerPercentage = 127; 									// Voltage of motors during auton
const float turningDiameter = 14.72455;										// Diameter of turn
const float deadzone = 8;													// Controller deadzone

double autonSelect = 2;

//===================\\

// == SENSOR DECLARATIONS == \\

pros::ADIDigitalIn armSwitch('E');




void tareWheels(){
	leftMotorFront.tare_position();
	leftMotorBack.tare_position();
	rightMotorFront.tare_position();
	rightMotorBack.tare_position();
}

void driveDist(double inches, double pct = autonPowerPercentage){
	double dist = inches / inchesPerDegree;
	dist *= slightFix;
	tareWheels();
	leftMotorFront.move_absolute(dist, pct);
	leftMotorBack.move_absolute(dist, pct);
	rightMotorFront.move_absolute(dist, pct);
	rightMotorBack.move_absolute(dist, pct);
}

void turn(double degrees, double pct = autonPowerPercentage){// 90 / 360 // 225 / 900
	degrees *= 2.5;
	float turningRatio = turningDiameter / wheelDiameter;
	float dist = turningRatio * degrees;
	dist *= slightFix * 1.6;
	tareWheels();
	leftMotorFront.move_absolute(dist, pct);
	leftMotorBack.move_absolute(dist, pct);
	rightMotorFront.move_absolute(-dist, pct);
	rightMotorBack.move_absolute(-dist, pct);
}

void moveTilesForward(double tiles){
	driveDist(tiles * 12);
}

inline void smallStop(){
	pros::delay(500);
}
void largeStop(){
	pros::delay(1000);
}
void largerStop(){
	pros::delay(4000);
}
void driveForward(){
	double volts = 15;
	leftMotorFront.move(volts);
	leftMotorBack.move(volts);
	rightMotorFront.move(volts);
	rightMotorBack.move(volts);
}
void driveBackward(){
	double volts = -15;
	leftMotorFront.move(volts);
	leftMotorBack.move(volts);
	rightMotorFront.move(volts);
	rightMotorBack.move(volts);
}
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;

	if(pressed){
		autonSelect++;
		if(autonSelect == 4){
			autonSelect = 1;
		}
		pros::lcd::print(2, "%s %d %s","Autonomous ", autonSelect, " selected"); 
	}else{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_RED);
	pros::lcd::register_btn0_cb(on_center_button);
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_RED);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::lcd::initialize();
	pros::lcd::set_background_color(LV_COLOR_RED);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */




void autonomous() {
	driveDist(2);
	pros::delay(400);

	// Slowly run into the roller more to account for bounce back
	driveForward();
	largeStop();
	
	// Roll
		roller.tare_position();
		roller.move_absolute(90 * 2.5, autonPowerPercentage);
		smallStop();

	
		

	

}

void opcontrol() {
	
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
		
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);


		// == Voltage Control == \\

		int leftStickVoltage = master.get_analog(ANALOG_LEFT_Y);								//Sets voltage to be the value of the joysticks y axis (Basic Tank Drive)
		int rightStickVoltage = master.get_analog(ANALOG_RIGHT_Y);
		int temp = leftStickVoltage;
		leftStickVoltage = -rightStickVoltage;
		rightStickVoltage = -temp;
		

		// == Basic Tank Drive == \\

		if(leftStickVoltage > deadzone || leftStickVoltage < -deadzone){ 						//Deadzone of 8 
			if(master.get_digital(DIGITAL_UP)){ 												//If up is pushed, half speed drive
				leftStickVoltage = leftStickVoltage/2;
			}
			leftMotorFront.move(leftStickVoltage);
			leftMotorBack.move(leftStickVoltage);
		}else{ 																					//If nothing is pressed, wheels should not run
			leftMotorFront.brake();
			leftMotorBack.brake();
		}
		
		if(rightStickVoltage > deadzone || rightStickVoltage < -deadzone){ 						//Deadzone of 8
			if(master.get_digital(DIGITAL_UP)){ 												//If up is pressed, half speed drive
				rightStickVoltage = rightStickVoltage/2;
			}
			rightMotorFront.move(rightStickVoltage);
			rightMotorBack.move(rightStickVoltage);
		}else{ 																					//If nothing is pressed, wheels should not run
			rightMotorBack.brake();
			rightMotorFront.brake();
		}
		


		// == CATAPULT == \\

		if(armSwitch.get_value()){										// Arm switch keeps catapult permanently loaded
			if(master.get_digital(DIGITAL_L1)){							// If L1 (Left Bumper) is hit, shoot catapult
				catapult.move(127);
			}else{
				catapult.move(15);										// Keep constant voltage on motor to counteract rubber bands
			}
		}else{
			catapult.move(100);											//If switch is not hit, move catapult into loaded position
		}





		// == INTAKE == \\ 

		if(master.get_digital(DIGITAL_R1)){ //If r1 is pushed, intake.
			if(master.get_digital(DIGITAL_UP)){ //If up arrow is pushed at the same time, half speed intake
				intake.move(64);
			}else{
				intake.move(127);
			}
		}else if(master.get_digital(DIGITAL_R2)){ //If L1 is pushed, outtake
			if(master.get_digital(DIGITAL_UP)){ //If up arrow is pushed at the same time, half speed outtake.
				intake.move(-64);
			}else{
				intake.move(-127);
			}
		}else{
			intake.brake(); //if nothing is pushed, intake should be stopped
		}

		

		// == EXPANSION CODE == \\

		// if(master.get_digital(DIGITAL_B)){
		// 	expansion.move(127);
		// }else if(master.get_digital(DIGITAL_A)){
		// 	expansion.move(-127);
		// }else{
		// 	expansion.brake();
		// }


		// == COLOR ROLLER == \\ 

		if(master.get_digital(DIGITAL_L2)){								// If L2 is pressed, spin the roller
			roller.move(127);
		}else{
			roller.brake();
		}


		// == TELEMETRY == \\

		pros::delay(2);
	}
}