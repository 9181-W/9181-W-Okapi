#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

#define ANALOG_LED_PORT 1
#define ADI_GYRO_PORT 2

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void led_fn(void* param)
{
  //Pwm_out.state(100,vex::percentUnits::pct);
  pros::ADIAnalogOut led_strip(ANALOG_LED_PORT);
  led_strip.set_value(4095); // Set the port to 5V

  // For Later Usage
  while (true)
  {
    pros::delay(1000);
  }
}

ADIGyro gyro(ADI_GYRO_PORT);

double get_proper_gyro()
{
  return gyro.get() / 10.0 * 0.96749255;
}

extern okapi::ChassisControllerPID chassis;

void gyro_turn(double angle)
{
  double target = 0.0;
  double power = 0.0;
  double current = 0.0;
  double error = 0.0;
  double time_elapsed = 0.0;
  double last_error = 0.0;
  double derivative = 0.0;
  const double EMERGENCY_SHUT_DOWN = 2100;
  const double ALLOWABLE_ERROR = 1.0;
  const double MAX_SPEED = 0.5;
  const double MIN_SPEED = -0.5;
  const double LOWEST_MOTOR_SPEED_THAT_STILL_TURNS = 0.1;
  const double kp = 0.0075;
  const double kd = 0.0015;

  //TUNE KP AND KD WHILE LMSTST = 0
  //MAKE SURE THAT GYRO VALUE = 87 - 89
  //TUNE LMSTST


  gyro.reset();
  target = angle;
	do
	{
		current = get_proper_gyro();	// Use JPearman's Gyro Library to filter and scale gyro readings

    pros::lcd::print(1, "Current Gyro Value: %f", current);

    error = target - current;

    derivative = last_error - error;

    last_error = error;

    pros::lcd::print(2, "Current Error: %f", error);

		power	= kp * error - kd * derivative;			// Calculate power level
    pros::lcd::print(3, "Power: %f", power);

		if(power > MAX_SPEED)
    {
			power = MAX_SPEED;
    }
		if(power < MIN_SPEED)
    {
			power = MIN_SPEED;
    }
    if((power < LOWEST_MOTOR_SPEED_THAT_STILL_TURNS) && (power > 0))
    {
      power = LOWEST_MOTOR_SPEED_THAT_STILL_TURNS;
    }
    pros::lcd::print(4, "Power Clamped: %f", power);

    chassis.arcade(0, power);

    if (time_elapsed > EMERGENCY_SHUT_DOWN)
    {
      break;
    }

    pros::delay(10);

    time_elapsed = time_elapsed + 10;

	}
	while (abs(error) > ALLOWABLE_ERROR);
  chassis.arcade(0, 0);

  while (true)
  {
        pros::lcd::print(6, "Gyro Value: %f", get_proper_gyro());
        pros::delay(10);
  }
}

void initialize()
{

  pros::lcd::initialize();
  pros::lcd::print(0, "HELLO WORLD");

  pros::Task my_task (led_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "My Task");

  pros::delay(1500);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
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
void competition_initialize()
{
}
