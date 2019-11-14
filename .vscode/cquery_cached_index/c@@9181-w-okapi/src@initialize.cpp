#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

#define ANALOG_LED_PORT 1
#define ADI_GYRO_PORT 2


extern okapi::Motor lf_mtr;
extern okapi::Motor lr_mtr;
extern okapi::Motor rf_mtr;
extern okapi::Motor rr_mtr;


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

void gyro_turn(QAngle q_angle);

ADIGyro gyro(ADI_GYRO_PORT);

double get_proper_gyro()
{
  return gyro.get() / 10.0 * 0.9531;
}

void gyro_turn_to(QAngle q_angle)
{
  gyro_turn(q_angle);
  gyro.reset();
}


extern okapi::ChassisControllerIntegrated chassis;

void gyro_turn(QAngle q_angle)
{

  //Comment Out These Three Lines
  //chassis.turnAngleAsync(q_angle);
  //chassis.waitUntilSettled();
  //return;

  double angle = q_angle.convert(degree);
  double target = 0.0;
  double power = 0.0;
  double current = 0.0;
  double error = 0.0;
  double time_elapsed = 0.0;
  double last_error = 0.0;
  double derivative = 0.0;
  const double EMERGENCY_SHUT_DOWN = 10000;
  const double ALLOWABLE_ERROR = 1.0;
  const double MAX_SPEED = 0.5;
  const double MIN_SPEED = -0.5;
  const double LOWEST_MOTOR_SPEED_THAT_STILL_TURNS = 0.04;
  const double kp = 0.004;
  const double kd = 0.000;

  chassis.setMaxVelocity(200);

  //TUNE KP AND KD WHILE LMSTST = 0
  //MAKE SURE THAT GYRO VALUE = 87 - 89
  //TUNE LMSTST

  //gyro.reset();
  target = angle;
	do
	{
		current = get_proper_gyro();

    pros::lcd::print(1, "Current Gyro Value: %f", current);

    error = target - current;

    derivative = error - last_error;

    last_error = error;

    pros::lcd::print(2, "Current Error: %f", error);

		power	= kp * error + kd * derivative;			// Calculate power level
    pros::lcd::print(3, "Power: %f", power);

		if(power > MAX_SPEED)
    {
			power = MAX_SPEED;
    }
		if(power < MIN_SPEED)
    {
			power = MIN_SPEED;
    }

    if ((power > 0) && (power < LOWEST_MOTOR_SPEED_THAT_STILL_TURNS))
    {
       power = LOWEST_MOTOR_SPEED_THAT_STILL_TURNS;
    }

    else if ((power < 0) && (power > -LOWEST_MOTOR_SPEED_THAT_STILL_TURNS))
    {
      power = -LOWEST_MOTOR_SPEED_THAT_STILL_TURNS;
    }

    pros::lcd::print(4, "Power Clamped: %f", power);


    //power = power * 200;

/*
    lf_mtr.moveVelocity(power);
    lr_mtr.moveVelocity(-power);
    rf_mtr.moveVelocity(-power);
    rr_mtr.moveVelocity(power);
*/

    if (time_elapsed > EMERGENCY_SHUT_DOWN)
    {
      pros::lcd::print(6, "Emergency Exit");
      break;
    }

    static int error_count = 0;
    if (abs(error) < ALLOWABLE_ERROR)
    {
      error_count++;
      if (error_count > 1)
      {
        pros::lcd::print(6, "Normal Stop: %f",error);
        break;
      }
      power = 0;
    }
    else
    {
      error_count = 0;
    }

    chassis.rotate(power);
    pros::delay(20);
    time_elapsed = time_elapsed + 20;

	}
	while (true);

/*
  lf_mtr.move(0);
  lr_mtr.move(0);
  rf_mtr.move(0);
  rr_mtr.move(0);
*/

  chassis.stop();
  //chassis.resetSensors();


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
