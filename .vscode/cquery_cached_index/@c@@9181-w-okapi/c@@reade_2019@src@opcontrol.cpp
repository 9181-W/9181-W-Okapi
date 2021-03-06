#include "main.h"

//void __sync_synchronize(void) {
//    __sync_synchronize();
//}
extern "C" void __sync_synchronize() {}

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
#define LEFT_FRONT_WHEEL_PORT 1
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define RIGHT_REAR_WHEEL_PORT 20
#define TRAY_MOTOR_PORT 15
#define ARM_MOTOR_PORT 16
#define LEFT_INTAKE_MOTOR_PORT 8
#define RIGHT_INTAKE_MOTOR_PORT 2
#define SCALE 1.0

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor lf_mtr(LEFT_FRONT_WHEEL_PORT);
pros::Motor lr_mtr(LEFT_REAR_WHEEL_PORT);
pros::Motor rf_mtr(RIGHT_FRONT_WHEEL_PORT, true);
pros::Motor rr_mtr(RIGHT_REAR_WHEEL_PORT, true);
pros::Motor a_mtr(ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
pros::Motor t_mtr(TRAY_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true);
pros::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT);
pros::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT, true);

// Forward declarations
void arcade_control_1();
void arcade_control_2();
void arcade_control_3();
void tank_control();

void opcontrol()
{

  lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  a_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  t_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  li_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ri_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while (true)
	{
    //arcade_control_1();
	  //arcade_control_2();
	  arcade_control_3();
	  //tank_control();

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

		// make arm move
		if (master.get_digital(DIGITAL_R2) == 1)
		{
			t_mtr.move(45)
		}
		else if (master.get_digital(DIGITAL_L2) == 1)
		{
			t_mtr.move(-70);
		}
		else
		{
			t_mtr.move(0);
		}

    if (master.get_digital(DIGITAL_B))
    {
      t_mtr.move(-127);
    }

    //Use Y button to open cartridge
    static bool do_once = true;
    if ((master.get_digital(DIGITAL_Y) == 1) && do_once)
    {
      t_mtr.move(127);
      pros::delay(400);
      t_mtr.move(0);
      pros::delay(400);
      t_mtr.move(-127);
      pros::delay(400);
      t_mtr.move(0);
      do_once = false;
    }

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

void arcade_control_1()
{
  int power = master.get_analog(ANALOG_LEFT_Y);
  int turn = master.get_analog(ANALOG_RIGHT_X);
  int left = power;
  int right = power;

	if (turn < -64)
	{
		left = power * -1;
	}

	if (turn > 64)
	{
		right = power * -1;
	}

	left = left * SCALE;
	right = right * SCALE;

  lf_mtr.move(left);
  lr_mtr.move(left);
	rf_mtr.move(right);
	rr_mtr.move(right);
}


void arcade_control_2()
{
	int power = master.get_analog(ANALOG_LEFT_Y);
	int turn = master.get_analog(ANALOG_RIGHT_X);
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

	lf_mtr.move(left);
	lr_mtr.move(left);
	rf_mtr.move(right);
	rr_mtr.move(right);
}


void arcade_control_3()
{
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

	lf_mtr.move(left);
	lr_mtr.move(left);
	rf_mtr.move(right);
	rr_mtr.move(right);
}


void tank_control()
{
	// Read the controller
  float left = master.get_analog(ANALOG_LEFT_Y);
	float right = master.get_analog(ANALOG_RIGHT_Y);

  // Set the speed of the motors.
	left = left * SCALE;
	right = right * SCALE;
  lf_mtr.move(left);
	lr_mtr.move(left);
	rf_mtr.move(right);
  rr_mtr.move(right);
}
