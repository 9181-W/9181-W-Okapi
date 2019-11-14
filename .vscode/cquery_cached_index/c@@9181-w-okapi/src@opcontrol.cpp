#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.5i
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
#define SCALE 1.0

pros::Controller master(pros::E_CONTROLLER_MASTER);

// Forward declarations
void arcade_control();

extern okapi::Motor a_mtr;
extern okapi::Motor t_mtr;
extern okapi::Motor li_mtr;
extern okapi::Motor ri_mtr;
extern okapi::Motor lf_mtr;
extern okapi::Motor lr_mtr;
extern okapi::Motor rf_mtr;
extern okapi::Motor rr_mtr;

void opcontrol()
{

  lf_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
  lr_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
  rf_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
  rr_mtr.setBrakeMode(AbstractMotor::brakeMode::coast);
  t_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  a_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  li_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  ri_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);

	while (true)
	{
	  arcade_control();

		//Use buttons to make intake move
		if (master.get_digital(DIGITAL_R1) == 1)
		{
			li_mtr.move(127);
			ri_mtr.move(127);
		}
		else if (master.get_digital(DIGITAL_L1) == 1)
		{
			li_mtr.move(-40);
			ri_mtr.move(-40);
		}
		else
		{
			li_mtr.move(0);
			ri_mtr.move(0);
		}

//MAKE TRAY MOVE
		if (master.get_digital(DIGITAL_R2) == 1)
		{
      t_mtr.moveVelocity(20);
    }
		else if (master.get_digital(DIGITAL_L2) == 1)
		{
			t_mtr.moveVelocity(-70);
		}
		else
		{
			t_mtr.moveVelocity(0);
		}

    /*
    if (master.get_digital(DIGITAL_B))
    {
      t_mtr.moveVelocity(-100);
    }
    */

    if (master.get_digital(DIGITAL_X))
    {
      t_mtr.move(0);
      a_mtr.move(0);
      li_mtr.move(0);
      ri_mtr.move(0);
      lf_mtr.move(0);
      lr_mtr.move(0);
      rf_mtr.move(0);
      rr_mtr.move(0);
      t_mtr.moveVelocity(100);
      pros::delay(420);
      t_mtr.move(0);
      a_mtr.move(0);
      li_mtr.move(0);
      ri_mtr.move(0);
      lf_mtr.move(0);
      lr_mtr.move(0);
      rf_mtr.move(0);
      rr_mtr.move(0);
    }


    /*
    //Use Y button to open cartridge
    static bool do_once = true;
    if ((master.get_digital(DIGITAL_Y) == 1) && do_once )
    {
      t_mtr.move(0);
      a_mtr.move(0);
      li_mtr.move(0);
      ri_mtr.move(0);
      lf_mtr.move(0);
      lr_mtr.move(0);
      rf_mtr.move(0);
      rr_mtr.move(0);

      t_mtr.move(1400);
      t_mtr.move(-1400);
      a_mtr.move(1400);
      a_mtr.move(-1400);

      t_mtr.move(0);
      t_mtr.move(0);
      a_mtr.move(0);
      li_mtr.move(0);
      ri_mtr.move(0);
      lf_mtr.move(0);
      lr_mtr.move(0);
      rf_mtr.move(0);
      rr_mtr.move(0);
      do_once = false;
    }
    */

		//Use buttons to set brake mode or coast
		if (master.get_digital(DIGITAL_RIGHT) == 1)
		{
			printf("Press Right");
			lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			rr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			lf_mtr.move(0);
			lr_mtr.move(0);
			rf_mtr.move(0);
			rr_mtr.move(0);
		}
    else
    {
      printf("Right not pressed");
    }
		if (master.get_digital(DIGITAL_DOWN) == 1)
		{
			printf("Press DOWN");
			lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			rr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
			lf_mtr.move(0);
			lr_mtr.move(0);
			rf_mtr.move(0);
			rr_mtr.move(0);
		}

		pros::delay(2);
	}
}


void arcade_control()
{
  int upwards = master.get_analog(ANALOG_RIGHT_Y);
  int power = master.get_analog(ANALOG_LEFT_Y);
	int turn = master.get_analog(ANALOG_LEFT_X);
	int left = power + turn;
	int right = power - turn;

  if (left > 127)
	{
	  left = 127;
	}

	if (left < -127)
	{
		left = -127;
	}

	if (right > 127)
	{
		right = 127;
	}

	if (right < -127)
	{
		right = -127;
	}

	left = left * SCALE;
	right = right * SCALE;
  upwards = upwards * SCALE;

  a_mtr.move(upwards);
	lf_mtr.move(left);
	lr_mtr.move(left);
	rf_mtr.move(right);
	rr_mtr.move(right);
}
