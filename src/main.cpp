// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include<list>

using namespace pros;
using namespace std;


// void resetEncoders() { //we can't add this to main.h because main.h doesn't
// refer to robot.h (where LF, LB, etc. are located) 	LF.tare_position(); //or
// set_zero_position(0) or set_zero_position(LF.get_position()); (sets current
// encoder position to 0) 	LB.tare_position(); 	RF.tare_position();
// 	RB.tare_position();
// }

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
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
  OpticalC.set_led_pwm(100);
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  ODOMY.reset_position();
  ODOMX.reset_position();

  // pros::lcd::register_btn1_cb(on_center_button);
  // optical.set_led_pwm(100);


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

 

int atn = 1;
int RingColor = 2;
int pressed = 0;
string autstr;
float errorp;
int time3;

vector<int> leftSideVoltage = {};
vector<int> RightSideVoltage = {};
vector<int> leftSideEnc = {};
vector<int> RightSideEnc = {};
vector<int> angleList = {};





 
void competition_initialize() {

}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
int headingCorrection = 0;
int left = 0;
int right = 0;

setConstants(1, 0, 1); // left side pid values
setConstants2(1, 0, 1); //right side pid values
setConstants3(1, 0, 1); //heading correction values


	while (true) {

    headingCorrection = calcPID3(angleList[time3], imu.get_rotation(), 0, 0);
    left = leftSideVoltage[time3] + calcPID(leftSideEnc[time3], LF.get_position(), 0, 0) + headingCorrection;
    right = RightSideVoltage[time3] + calcPID(RightSideEnc[time3], LF.get_position(), 0, 0) + headingCorrection;
    LF.move(left);
    LM.move(left);
    LB.move(left);
    RF.move(right);
    RM.move(right);
    RB.move(right);

	  	time3 += 1;
		  delay(1);

	  }
  }