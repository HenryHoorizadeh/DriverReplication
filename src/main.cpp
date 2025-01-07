// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "odometry.h"
#include "pure_pursuit.h"
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

list<int> leftSideVoltage = {};
list<int> RightSideVoltage = {};
list<int> leftSideEnc = {};
list<int> RightSideEnc = {};
list<int> angleList = {};





 
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
  int cycle = 0;
  int time = 0;
  bool NEWL1 = false;
  bool NEWL2 = false;
  bool NEWR2 = false;
  bool NEWR1 = false;
  bool arcToggle = false;
  bool tankToggle = true;
  bool mogoToggle = true;
  bool intakeToggle = false;
  bool scrapperToggle = false;
  bool hangToggle = false;
  bool liftToggle = false;
  double maxRPM = 0;
  double motorTotal = 0;
  double avgRPM = 0;
  double liftAngle = 0; 
  double rotoAngle = 0;
  float xvelo = 0;


	while (true) {


		//chassis arcade drive
		int power = con.get_analog(ANALOG_LEFT_Y); //power is defined as forward or backward
		int RX = con.get_analog(ANALOG_RIGHT_X); //turn is defined as left (positive) or right (negative)

    //int turn = int(RX); // Normal Rates
		//int turn = int(abs(RX) * RX / 127); //X Squared Rates
   int turn = int(pow(RX, 3) / pow(127, 2)); //X Cubed Rates
		int left = power + turn;
		int right = power - turn;

    // //switch between arcade and tank
    if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      arcToggle = !arcToggle;
      tankToggle = !tankToggle;
    }


   
    if (tankToggle) {
      LF.move(con.get_analog(ANALOG_LEFT_Y));
      LM.move(con.get_analog(ANALOG_LEFT_Y));
      LB.move(con.get_analog(ANALOG_LEFT_Y));
      RF.move(con.get_analog(ANALOG_RIGHT_Y));
      RM.move(con.get_analog(ANALOG_RIGHT_Y));
      RB.move(con.get_analog(ANALOG_RIGHT_Y));
    }
    if (arcToggle) {
      LF.move(left);
      LM.move(left);
      LB.move(left);
      RF.move(right);
      RM.move(right);
      RB.move(right);
    }

  leftSideVoltage.push_front(con.get_analog(ANALOG_LEFT_Y));
  RightSideVoltage.push_front(con.get_analog(ANALOG_RIGHT_Y));
  leftSideEnc.push_front(LF.get_position());
  RightSideEnc.push_front(RF.get_position());
  angleList.push_front(imu.get_rotation());



    if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){

      std::cout << "leftSideVoltage:" << std::endl;
      for (int x : leftSideVoltage) {
        std::cout << x << ", ";
      }

      std::cout << "RightSideVoltage:" << std::endl;
      for (int x : RightSideVoltage) {
        std::cout << x << ", ";
      }

      std::cout << "leftSideEnc:" << std::endl;
      for (int x : leftSideEnc) {
        std::cout << x << ", ";
      }

      std::cout << "RightSideEnc:" << std::endl;
      for (int x : RightSideEnc) {
        std::cout << x << ", ";
      }

      std::cout << "angleList:" << std::endl;
      for (int x : angleList) {
        std::cout << x << ", ";
      }

    }

	  	time += 1;
		  delay(1);

	  }
  }