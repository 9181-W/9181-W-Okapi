#include "main.h"
#include "okapi/api.hpp"
using namespace okapi;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

#define LEFT_FRONT_WHEEL_PORT 1
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define RIGHT_REAR_WHEEL_PORT 20
#define TRAY_MOTOR_PORT 15
#define ARM_MOTOR_PORT 16
#define LEFT_INTAKE_MOTOR_PORT 8
#define RIGHT_INTAKE_MOTOR_PORT 2

#define MEGA_MAX_SPEED (100.0)
#define MAXER_SPEED (127.0)
#define MAXISH_SPEED (90.0)
#define MAX_SPEED (80.0)
#define THREE_QUARTER_SPEED (60.0)
#define HALF_SPEED (40.0)
#define SLOW_SPEED (20.0)

#define LIFT_POSITION_BOTTOM 0.0
#define LIFT_POSITION_MIDDLE 2000.0
#define LIFT_POSITION_TOP    4000.0

extern pros::Motor a_mtr;
extern pros::Motor t_mtr;
extern pros::Motor li_mtr;
extern pros::Motor ri_mtr;

auto chassis = ChassisControllerFactory::create
(
  {LEFT_FRONT_WHEEL_PORT, LEFT_REAR_WHEEL_PORT},
  {-RIGHT_FRONT_WHEEL_PORT, -RIGHT_REAR_WHEEL_PORT},
  IterativePosPIDController::Gains{0.005, 0, 0.0001},
  IterativePosPIDController::Gains{0.005, 0, 0.0001},
  IterativePosPIDController::Gains{0.005, 0, 0.0001},
  AbstractMotor::gearset::green,
  {4.0_in, 9_in}
);

void intake_on(double speed = 200.0)
{
  li_mtr.move_velocity(speed);
  ri_mtr.move_velocity(speed);
}
void intake_off()
{
  li_mtr.move_velocity(0);
  ri_mtr.move_velocity(0);
}

void autonomous()
{
  // Create controllers

  pros::Motor a_mtr(ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
  pros::Motor t_mtr(TRAY_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
  pros::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT);
  pros::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT, true);

  a_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  t_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  li_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ri_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


//BACK BLUE
/*
  chassis.setMaxVelocity(MEGA_MAX_SPEED);
  t_mtr.move(200);
  pros::delay(800);
  t_mtr.move(0);
  t_mtr.move(-200);
  pros::delay(200);
  a_mtr.move(200);
  pros::delay(1400);
  a_mtr.move(0);
  a_mtr.move(-200);
  pros::delay(1000);
  t_mtr.move(200);
  pros::delay(100);
  t_mtr.move(0);
  chassis.moveDistanceAsync(40_in);
  li_mtr.move(200);
  ri_mtr.move(200);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.turnAngle(28_deg);
  chassis.moveDistanceAsync(8_in);
  chassis.waitUntilSettled();
  li_mtr.move(200);
  ri_mtr.move(200);
  pros::delay(900);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(MAXISH_SPEED);
  chassis.turnAngle(215_deg);
  chassis.setMaxVelocity(MEGA_MAX_SPEED);
  chassis.moveDistanceAsync(37_in);
  chassis.waitUntilSettled();
  chassis.turnAngle(15_deg);
  //chassis.moveDistanceAsync(6_in);
  //chassis.waitUntilSettled();
  t_mtr.move(70);
  pros::delay(2300);
  t_mtr.move(0);
  chassis.moveDistanceAsync(-6_in);
  chassis.waitUntilSettled();
*/
//BACK RED
/*
    chassis.setMaxVelocity(MEGA_MAX_SPEED);
    t_mtr.move(200);
    pros::delay(800);
    t_mtr.move(0);
    t_mtr.move(-200);
    pros::delay(200);
    a_mtr.move(200);
    pros::delay(1400);
    a_mtr.move(0);
    a_mtr.move(-200);
    pros::delay(800);
    t_mtr.move(200);
    pros::delay(100);
    t_mtr.move(0);
    chassis.moveDistanceAsync(40_in);
    li_mtr.move(200);
    ri_mtr.move(200);
    chassis.waitUntilSettled();
    li_mtr.move(0);
    ri_mtr.move(0);
    chassis.turnAngle(-28_deg);
    chassis.moveDistanceAsync(8_in);
    chassis.waitUntilSettled();
    li_mtr.move(200);
    ri_mtr.move(200);
    pros::delay(900);
    li_mtr.move(0);
    ri_mtr.move(0);
    chassis.setMaxVelocity(MAXISH_SPEED);
    chassis.turnAngle(-215_deg);
    chassis.setMaxVelocity(MEGA_MAX_SPEED);
    chassis.moveDistanceAsync(36_in);
    chassis.waitUntilSettled();
    chassis.turnAngle(-20_deg);
    //chassis.moveDistanceAsync(6_in);
    //chassis.waitUntilSettled();
    t_mtr.move(70);
    pros::delay(2300);
    t_mtr.move(0);
    chassis.moveDistanceAsync(-6_in);
    chassis.waitUntilSettled();
*/

//FRONT
    chassis.setMaxVelocity(MAXER_SPEED);
  /*  t_mtr.move(200);
    pros::delay(800);
    t_mtr.move(0);
    t_mtr.move(-200);
    pros::delay(200);
    a_mtr.move(200);
    pros::delay(1400);
    a_mtr.move(0);
    a_mtr.move(-200);
    pros::delay(800);
    t_mtr.move(200);
    pros::delay(100);
    t_mtr.move(0);*/
    chassis.moveDistanceAsync(-11_in);
    chassis.waitUntilSettled();
    chassis.moveDistanceAsync(11_in);
    chassis.waitUntilSettled();


//SKILLS
/*
chassis.setMaxVelocity(MEGA_MAX_SPEED);
t_mtr.move(200);
pros::delay(800);
t_mtr.move(0);
t_mtr.move(-200);
pros::delay(200);
a_mtr.move(200);
pros::delay(1400);
a_mtr.move(0);
a_mtr.move(-200);
pros::delay(1000);
t_mtr.move(200);
pros::delay(100);
t_mtr.move(0);
chassis.moveDistanceAsync(40_in);
li_mtr.move(200);
ri_mtr.move(200);
chassis.waitUntilSettled();
li_mtr.move(0);
ri_mtr.move(0);
chassis.turnAngle(28_deg);
chassis.moveDistanceAsync(8_in);
chassis.waitUntilSettled();
li_mtr.move(200);
ri_mtr.move(200);
pros::delay(900);
li_mtr.move(0);
ri_mtr.move(0);
chassis.setMaxVelocity(MAXISH_SPEED);
chassis.turnAngle(215_deg);
chassis.setMaxVelocity(MEGA_MAX_SPEED);
chassis.moveDistanceAsync(36_in);
chassis.waitUntilSettled();
chassis.turnAngle(15_deg);
//chassis.moveDistanceAsync(6_in);
//chassis.waitUntilSettled();
t_mtr.move(60);
pros::delay(2400);
t_mtr.move(0);
chassis.moveDistanceAsync(-6_in);
chassis.waitUntilSettled();
*/

  /*
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(42_in);
  intake_on(200);
  chassis.waitUntilSettled();
  intake_off();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(8.5_in);
  intake_on(200);
  chassis.waitUntilSettled();
  intake_off();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(40_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(-7.4_in);
  intake_on(200);
  chassis.waitUntilSettled();
  intake_on(200);
  pros::delay(200);
  intake_off();
  pros::delay(400);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-53_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(10.5_in);
  intake_on(154);
  chassis.waitUntilSettled();
  intake_off();
  chassis.moveDistanceAsync(-9_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-173_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  intake_on(180);
  pros::delay(1600);
  intake_off();
  chassis.moveDistanceAsync(13_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-115_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(18.5_in);
  intake_on(200);
  chassis.waitUntilSettled();
  intake_off();
  */

//Upstairs Skills
/*
  chassis.setMaxVelocity(MAX_SPEED);

  la_mtr.move(127);
  ra_mtr.move(127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(400);
  la_mtr.move(-127);
  ra_mtr.move(-127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  chassis.moveDistanceAsync(42_in);
  li_mtr.move(150);
  ri_mtr.move(150);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(7.5_in);
  chassis.waitUntilSettled();
  li_mtr.move(140);
  ri_mtr.move(140);
  pros::delay(500);
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  */


/*
  chassis.setMaxVelocity(MEGA_MAX_SPEED);
  la_mtr.move(127);
  ra_mtr.move(127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(400);
  la_mtr.move(-127);
  ra_mtr.move(-127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(200);
  chassis.moveDistanceAsync(41_in);
  li_mtr.move(180);
  ri_mtr.move(180);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(THREE_QUARTER_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAXISH_SPEED);
  chassis.moveDistanceAsync(8.5_in);
  chassis.waitUntilSettled();
  li_mtr.move(150);
  ri_mtr.move(150);
  pros::delay(480);
  li_mtr.move(0);
  ri_mtr.move(0);
  li_mtr.move(-100);
  ri_mtr.move(-100);
  pros::delay(75);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(38_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(-12.25_in);
  chassis.waitUntilSettled();
  li_mtr.move(200);
  ri_mtr.move(200);
  pros::delay(800);
  li_mtr.move(0);
  ri_mtr.move(0);
  pros::delay(400);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-47_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(15.75_in);
  chassis.waitUntilSettled();
  li_mtr.move(200);
  ri_mtr.move(200);
  pros::delay(635);
  li_mtr.move(0);
  ri_mtr.move(0);
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(80);
  li_mtr.move(120);
  ri_mtr.move(120);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.moveDistanceAsync(-8_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-173_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(120);
  ri_mtr.move(120);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  li_mtr.move(200);
  ri_mtr.move(200);
  pros::delay(1150);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(16.25_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-105_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(18_in);
  chassis.waitUntilSettled();
  li_mtr.move(190);
  ri_mtr.move(190);
  pros::delay(500);
  li_mtr.move(0);
  ri_mtr.move(0);
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.moveDistanceAsync(-4_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(140_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(31_in);
  chassis.waitUntilSettled();
  ra_mtr.move(60);
  la_mtr.move(60);
  pros::delay(2000);
  la_mtr.move(0);
  ra_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.moveDistanceAsync(4.5_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(MEGA_MAX_SPEED);
  chassis.moveDistanceAsync(-10_in);
  chassis.waitUntilSettled();
  */


/*
  //Downstairs Back Blue
  chassis.setMaxVelocity(MAXISH_SPEED);
  la_mtr.move(127);
  ra_mtr.move(127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(400);
  la_mtr.move(-127);
  ra_mtr.move(-127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(200);
  chassis.moveDistanceAsync(41_in);
  li_mtr.move(160);
  ri_mtr.move(160);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(THREE_QUARTER_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAXISH_SPEED);
  chassis.moveDistanceAsync(10_in);
  li_mtr.move(150);
  ri_mtr.move(150);
  chassis.waitUntilSettled();
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.moveDistanceAsync(-20_in);
  chassis.waitUntilSettled();
  chassis.turnAngle(182_deg);
  chassis.moveDistanceAsync(19_in);
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.waitUntilSettled();
  ra_mtr.move(80);
  la_mtr.move(80);
  pros::delay(400);
  la_mtr.move(0);
  ra_mtr.move(0);
  pros::delay(200);
  ra_mtr.move(80);
  la_mtr.move(80);
  pros::delay(1200);
  la_mtr.move(0);
  ra_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.moveDistanceAsync(4_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(MEGA_MAX_SPEED);
  chassis.moveDistanceAsync(-10_in);
  chassis.waitUntilSettled();
  */


/*
    //Upstairs Back Red
    chassis.setMaxVelocity(MAXISH_SPEED);
    la_mtr.move(127);
    ra_mtr.move(127);
    pros::delay(400);
    ra_mtr.move(0);
    la_mtr.move(0);
    pros::delay(400);
    la_mtr.move(-127);
    ra_mtr.move(-127);
    pros::delay(400);
    ra_mtr.move(0);
    la_mtr.move(0);
    pros::delay(200);
    chassis.moveDistanceAsync(41_in);
    li_mtr.move(160);
    ri_mtr.move(160);
    chassis.waitUntilSettled();
    li_mtr.move(0);
    ri_mtr.move(0);
    chassis.setMaxVelocity(THREE_QUARTER_SPEED);
    chassis.turnAngle(35_deg);
    chassis.setMaxVelocity(MAXISH_SPEED);
    chassis.moveDistanceAsync(10_in);
    li_mtr.move(150);
    ri_mtr.move(150);
    chassis.waitUntilSettled();
    li_mtr.move(-75);
    ri_mtr.move(-75);
    pros::delay(100);
    li_mtr.move(100);
    ri_mtr.move(100);
    pros::delay(100);
    li_mtr.move(0);
    ri_mtr.move(0);
    chassis.moveDistanceAsync(-20_in);
    chassis.waitUntilSettled();
    chassis.turnAngle(179_deg);
    chassis.moveDistanceAsync(21_in);
    li_mtr.move(-75);
    ri_mtr.move(-75);
    pros::delay(100);
    li_mtr.move(100);
    ri_mtr.move(100);
    pros::delay(100);
    li_mtr.move(0);
    ri_mtr.move(0);
    chassis.waitUntilSettled();
    //chassis.turnAngle(15_deg);
    chassis.waitUntilSettled();
    ra_mtr.move(80);
    la_mtr.move(80);
    pros::delay(400);
    la_mtr.move(0);
    ra_mtr.move(0);
    pros::delay(200);
    ra_mtr.move(80);
    la_mtr.move(80);
    pros::delay(1200);
    la_mtr.move(0);
    ra_mtr.move(0);
    chassis.setMaxVelocity(HALF_SPEED);
    chassis.moveDistanceAsync(4_in);
    chassis.waitUntilSettled();
    chassis.setMaxVelocity(MEGA_MAX_SPEED);
    chassis.moveDistanceAsync(-10_in);
    chassis.waitUntilSettled();
    */

/*
      //Upstairs Back Blue
      chassis.setMaxVelocity(MEGA_MAX_SPEED);
      la_mtr.move(127);
      ra_mtr.move(127);
      pros::delay(400);
      ra_mtr.move(0);
      la_mtr.move(0);
      pros::delay(400);
      la_mtr.move(-127);
      ra_mtr.move(-127);
      pros::delay(400);
      ra_mtr.move(0);
      la_mtr.move(0);
      pros::delay(200);
      chassis.moveDistanceAsync(41_in);
      li_mtr.move(180);
      ri_mtr.move(180);
      chassis.waitUntilSettled();
      li_mtr.move(0);
      ri_mtr.move(0);
      chassis.setMaxVelocity(THREE_QUARTER_SPEED);
      chassis.turnAngle(-35_deg);
      chassis.setMaxVelocity(MAXISH_SPEED);
      chassis.moveDistanceAsync(10_in);
      li_mtr.move(150);
      ri_mtr.move(150);
      chassis.waitUntilSettled();
      li_mtr.move(-75);
      ri_mtr.move(-75);
      pros::delay(100);
      li_mtr.move(100);
      ri_mtr.move(100);
      pros::delay(100);
      li_mtr.move(0);
      ri_mtr.move(0);
      chassis.moveDistanceAsync(-20_in);
      chassis.waitUntilSettled();
      chassis.turnAngle(-179_deg);
      chassis.moveDistanceAsync(22.25_in);
      li_mtr.move(-75);
      ri_mtr.move(-75);
      pros::delay(100);
      li_mtr.move(100);
      ri_mtr.move(100);
      pros::delay(100);
      li_mtr.move(0);
      ri_mtr.move(0);
      chassis.waitUntilSettled();
      //chassis.turnAngle(15_deg);
      chassis.waitUntilSettled();
      ra_mtr.move(80);
      la_mtr.move(80);
      pros::delay(400);
      la_mtr.move(0);
      ra_mtr.move(0);
      pros::delay(200);
      ra_mtr.move(80);
      la_mtr.move(80);
      pros::delay(1200);
      la_mtr.move(0);
      ra_mtr.move(0);
      chassis.setMaxVelocity(HALF_SPEED);
      chassis.moveDistanceAsync(4_in);
      chassis.waitUntilSettled();
      chassis.setMaxVelocity(MEGA_MAX_SPEED);
      chassis.moveDistanceAsync(-10_in);
      chassis.waitUntilSettled();

*/

/*
  //blue/red front autonomous
  chassis.setMaxVelocity(THREE_QUARTER_SPEED);
  la_mtr.move(127);
  ra_mtr.move(127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  pros::delay(400);
  la_mtr.move(-127);
  ra_mtr.move(-127);
  pros::delay(400);
  ra_mtr.move(0);
  la_mtr.move(0);
  chassis.moveDistanceAsync(-9_in);
  chassis.waitUntilSettled();
  chassis.moveDistanceAsync(9_in);
  chassis.waitUntilSettled();
*/



}
