#include "gyro.h"
#include "autos.h"
#include "main.h"
#include "utils.h"
#include "okapi/api.hpp"
using namespace okapi;

void testing_nine_diff_stack(double start_time_x)
{

  intake_on(-60);
  a_mtr.move(100);
  pros::delay(400);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  double last_time = start_time_x;
  double time_sync = pros::c::millis();
  pros::lcd::print(0,"Deploy Time %f", time_sync - last_time);
  async_gyro_drive(chassis, 20_in, 80);
  intake_on();
  pros::delay(400);
  intake_off();
  alliance_tower();
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(600);
  intake_off();
  bottom();
  pros::delay(500);
  async_gyro_drive(chassis, -10_in, 80);
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  wait_for_drive_complete();
  gyro_turn(chassis, 48_deg);
  last_time = time_sync;
  time_sync = pros::c::millis();
  pros::lcd::print(1,"1st Tower %f", time_sync - last_time);
  //async_gyro_drive(chassis, -13_in, 90);
  //wait_for_drive_complete();
  async_gyro_drive(chassis, 115_in, 50);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  last_time = time_sync;
  time_sync = pros::c::millis();
  pros::lcd::print(2,"Picked Up Cubes %f", time_sync - last_time);
  intake_on();
  gyro_turn(chassis, -110_deg, 100, 22.5);
  async_gyro_drive(chassis, -20_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, 45_deg, 100, 22.5);
  gyro_drive(chassis, 30_in, 80, false);
  intake_off();
  edited_nine_cube_place();
  last_time = time_sync;
  time_sync = pros::c::millis();
  pros::lcd::print(3,"Stacked %f", time_sync - last_time);
  gyro_turn(chassis, -95_deg, 100, 22.5);
  async_gyro_drive(chassis, -6_in, 80);
  wait_for_drive_complete();
  async_gyro_drive(chassis, 16_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, -89.5_deg, 100, 22.5);
  async_gyro_drive(chassis, 20_in, 50);
  intake_on(115);
  wait_for_drive_complete();
  intake_off();
  medium_high_tower();
  gyro_drive(chassis, 6_in, 80);
  intake_on(-150);
  pros::delay(500);
  intake_off();
  async_gyro_drive(chassis, -6_in, 60);
  wait_for_drive_complete();
  gyro_turn(chassis, 90_deg);
  async_gyro_drive(chassis, -25_in, 60);
  bottom();
  wait_for_drive_complete();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  last_time = time_sync;
  time_sync = pros::c::millis();
  pros::lcd::print(4,"2nd Tower %f", time_sync - last_time);
  async_gyro_drive(chassis, 123_in, 50);
  intake_on();
  wait_for_drive_complete();
  gyro_drive(chassis, -2_in, 80, false);
  gyro_turn(chassis, 65_deg, 100, 22.5);
  gyro_drive(chassis, 35.5_in, 80, false);
  intake_off();
  edited_seven_cube_place();
  last_time = time_sync;
  time_sync = pros::c::millis();
  pros::lcd::print(1,"Picked Up Cubes %f", time_sync - last_time);
  async_gyro_drive(chassis, -7_in, 90);
  wait_for_drive_complete();
  gyro_turn(chassis, 90_deg, 100, 22.5);
  async_gyro_drive(chassis, -6_in, 90);
  wait_for_drive_complete();
  async_gyro_drive(chassis, 48_in, 90);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  async_gyro_drive(chassis, -4_in, 90);
  intake_on(-30);
  wait_for_drive_complete();
  intake_off();
  alliance_tower();
  intake_on(-150);
  pros::delay(500);
  intake_off();


}

void testing_new()
{
  forty_five_deg_turn();
}

void testing_nine_same_stack()
{
  intake_on(-60);
  a_mtr.move(60);
  pros::delay(1000);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 20_in, 80);
  intake_on();
  pros::delay(400);
  intake_off();
  alliance_tower();
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(600);
  intake_off();
  bottom();
  pros::delay(500);
  async_gyro_drive(chassis, -14_in, 80);
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  wait_for_drive_complete();
  gyro_turn(chassis, 45_deg);
  async_gyro_drive(chassis, -13_in, 90);
  wait_for_drive_complete();
  async_gyro_drive(chassis, 123_in, 50);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  intake_on();
  async_gyro_drive(chassis, -5_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 13_in, 80);
  wait_for_drive_complete();
  gyro_reset();
  async_gyro_drive(chassis, -6_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, 45_deg, 100, 22.5);
  async_gyro_drive(chassis, 13.5_in, 80);
  wait_for_drive_complete();
  nine_cube_place();
}

void new_red_back_port_5()
{
  intake_on(-60);
  a_mtr.move(100);
  pros::delay(400);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 20_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 20_in, 100);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -50_deg, 100, 22.5);
  async_gyro_drive(chassis, 4_in, 100);
  intake_on(-200);
  wait_for_drive_complete();
  intake_off();
  async_gyro_drive(chassis, -6_in, 60);
  intake_on(-60);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 145_deg, 100, 22.5);
  async_gyro_drive(chassis, 23_in, 100);
  wait_for_drive_complete();
  gyro_turn(chassis, 91_deg, 100, 22.5);
  async_gyro_drive(chassis, 40_in, 100);
  intake_on(200);
  wait_for_drive_complete();
  intake_off();
  intake_on();
}

void new_blue_back_port_5()
{
  intake_on(-60);
  a_mtr.move(100);
  pros::delay(400);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 20_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 90_deg, 100, 22.5);
  async_gyro_drive(chassis, 20_in, 100);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 50_deg, 100, 22.5);
  async_gyro_drive(chassis, 4_in, 100);
  intake_on(-200);
  wait_for_drive_complete();
  intake_off();
  async_gyro_drive(chassis, -6_in, 60);
  intake_on(-60);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -145_deg, 100, 22.5);
  async_gyro_drive(chassis, 23_in, 100);
  wait_for_drive_complete();
  gyro_turn(chassis, -91_deg, 100, 22.5);
  async_gyro_drive(chassis, 40_in, 100);
  intake_on(200);
  wait_for_drive_complete();
  intake_off();
  intake_on();


}

void testing()
{
  intake_on();
  gyro_turn(chassis, 110_deg, 100, 22.5);
  async_gyro_drive(chassis, -20_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, -45_deg, 100, 22.5);
  gyro_drive(chassis, 30_in, 80, false);
  intake_off();
  nine_cube_place();
}

void test_nine_point()
{
  intake_on(-60);
  a_mtr.move(60);
  pros::delay(1000);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 48_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 24_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 30_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 45_deg, 100, 22.5);
  async_gyro_drive(chassis, 30_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
}

void blue_back_port_5()
{
  intake_on(-60);
  a_mtr.move(60);
  pros::delay(1000);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 2_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 90_deg, 100, 22.5);
  async_gyro_drive(chassis, 14_in, 100);
  intake_on(-60);
  wait_for_drive_complete();
  intake_off();
  intake_on(-70);
  pros::delay(500);
  intake_off();
  async_gyro_drive(chassis, -7_in, 55);
  intake_on(-70);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -92_deg, 100, 22.5);
  async_gyro_drive(chassis, 39_in, 55);
  intake_on(-100);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 44_in, 200);
  intake_on(200);
  wait_for_drive_complete();
  intake_off();
  intake_on(200);
}

void red_back_port_4()
{
  intake_on(-60);
  a_mtr.move(60);
  pros::delay(1000);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 2_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 14_in, 100);
  intake_on(-60);
  wait_for_drive_complete();
  intake_off();
  intake_on(-70);
  pros::delay(500);
  intake_off();
  async_gyro_drive(chassis, -7_in, 55);
  intake_on(-70);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 92_deg, 100, 22.5);
  async_gyro_drive(chassis, 39_in, 55);
  intake_on(-100);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 90_deg, 100, 22.5);
  async_gyro_drive(chassis, 44_in, 200);
  intake_on(200);
  wait_for_drive_complete();
  intake_off();
  intake_on(200);
}

void red_front_port_3()
{
  intake_on(-60);
  a_mtr.move(100);
  pros::delay(400);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 50_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  intake_on();
  pros::delay(500);
  intake_off();
  async_gyro_drive(chassis, -29_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 132_deg, 100, 22.5);
  async_gyro_drive(chassis, 18_in, 80);
  intake_on(-40);
  wait_for_drive_complete();
  intake_off();
  seven_cube_place();
}

void blue_front_port_2()
{
  intake_on(-60);
  a_mtr.move(100);
  pros::delay(400);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 50_in, 55);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  intake_on();
  pros::delay(500);
  intake_off();
  async_gyro_drive(chassis, -29_in, 80);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, -132_deg, 100, 22.5);
  async_gyro_drive(chassis, 18_in, 80);
  intake_on(-40);
  wait_for_drive_complete();
  intake_off();
  seven_cube_place();
}

void skills_port_1()
{
  intake_on(-60);
  a_mtr.move(60);
  pros::delay(1000);
  a_mtr.move(0);
  bottom();
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  intake_off();
  async_gyro_drive(chassis, 20_in, 80);
  intake_on();
  pros::delay(400);
  intake_off();
  alliance_tower();
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(600);
  intake_off();
  bottom();
  pros::delay(500);
  async_gyro_drive(chassis, -9.75_in, 80);
  arm.waitUntilSettled();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  wait_for_drive_complete();
  gyro_turn(chassis, 47_deg);
  async_gyro_drive(chassis, -13_in, 90);
  wait_for_drive_complete();
  async_gyro_drive(chassis, 123_in, 40);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  intake_on();
  async_gyro_drive(chassis, -5_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, -90_deg, 100, 22.5);
  async_gyro_drive(chassis, 13_in, 80);
  wait_for_drive_complete();
  gyro_reset();
  async_gyro_drive(chassis, -6_in, 80);
  wait_for_drive_complete();
  gyro_turn(chassis, 43_deg, 100, 22.5);
  async_gyro_drive(chassis, 13.5_in, 80);
  wait_for_drive_complete();
  nine_cube_place();
  gyro_turn(chassis, 139.25_deg);
  async_gyro_drive(chassis, 32_in, 50);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  async_gyro_drive(chassis, -5_in, 60);
  intake_on(-40);
  wait_for_drive_complete();
  intake_off();
  medium_high_tower();
  async_gyro_drive(chassis, 8_in, 60);
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(800);
  intake_off();
  gyro_turn_to(chassis, 0_deg);
  async_gyro_drive(chassis, -5_in, 60);
  wait_for_drive_complete();
  gyro_turn(chassis, 90_deg);
  async_gyro_drive(chassis, -21_in, 60);
  bottom();
  wait_for_drive_complete();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  async_gyro_drive(chassis, 17_in, 60);
  intake_on(135);
  wait_for_drive_complete();
  intake_off();
  gyro_turn(chassis, 76_deg);
  async_gyro_drive(chassis, 31_in, 60);
  //intake_on(-30);
  //pros::delay(500);
  //intake_off();
  alliance_tower();
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(600);
  intake_off();
  async_gyro_drive(chassis, -18_in, 60);
  wait_for_drive_complete();
  gyro_turn(chassis, -76_deg);
  async_gyro_drive(chassis, -25_in, 60);
  bottom();
  wait_for_drive_complete();
  a_mtr.move(-100);
  pros::delay(400);
  a_mtr.move(0);
  async_gyro_drive(chassis, 46_in, 50);
  intake_on();
  wait_for_drive_complete();
  intake_off();
  async_gyro_drive(chassis, -4_in, 60);
  intake_on(-40);
  wait_for_drive_complete();
  intake_off();
  alliance_tower();
  async_gyro_drive(chassis, 2_in, 60);
  wait_for_drive_complete();
  intake_on(-200);
  pros::delay(800);
  intake_off();
}
