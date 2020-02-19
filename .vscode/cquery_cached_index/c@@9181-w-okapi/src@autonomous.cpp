#include "utils.h"
#include "autos.h"
#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

void autonomous()
{

  pros::lcd::print(0, "autonomous");

  double start_time = pros::c::millis();

  init_auto();

  testing_new();

  //testing_nine_diff_stack(start_time);
  //blue_front_port_2();
  //new_blue_back_port_5();

  double end_time = pros::c::millis();
  pros::lcd::print(6,"autonomous Time %f", end_time - start_time);
}
