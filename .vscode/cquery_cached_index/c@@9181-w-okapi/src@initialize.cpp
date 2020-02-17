#include "main.h"
#include "okapi/api.hpp"
#include "gyro.h"
#include "inertial.h"
using namespace okapi;

#define ANALOG_LED_PORT 1


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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::print(0, "initialize");
  pros::lcd::print(5, "GOOD LUCK");

  //gyro_initialize();
  inertial_initialize();

  /*
  pros::Task my_task (led_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "My Task");
  */
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
  pros::lcd::print(0, "disabled");
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
  pros::lcd::print(0, "competition_initialize");
}
