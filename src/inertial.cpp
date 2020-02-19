#include "main.h"
#include "okapi/api.hpp"
#include "gyro.h"
#include "inertial.h"

#define INERTIAL_PORT 12

pros::Imu* inertial_A = NULL;
double current_inertial_value = 0.0;

void inertial_update(void* param)
{
  while(true)
  {
    current_inertial_value = inertial_A->get_rotation();
    pros::lcd::print(7,"Inertial Value %f",current_inertial_value);
    pros::delay(20);
  }
}

void inertial_initialize()
{
  inertial_A = new pros::Imu(INERTIAL_PORT);
  inertial_A->reset();

  double start_time = pros::c::millis();
  pros::lcd::print(7,"CALIBRATING");
  pros::delay(10);
  while(inertial_A->is_calibrating() == true)
  {
    pros::delay(50);
  }
  double end_time = pros::c::millis();
  pros::lcd::print(5,"Calibration Time %f", end_time - start_time);

  pros::Task inetrial_display_task (inertial_update, (void*)"PROSV5NEW", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Inertial Display Task");
}

double get_inertial_value()
{
  return current_inertial_value;
}

void inertial_reset()
{
  // NO - DO NOT DO THIS- inertial_A->reset();
  // What you will do here is set the current value as zero.
}
