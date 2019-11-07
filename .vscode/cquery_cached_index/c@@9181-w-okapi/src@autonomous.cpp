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

#define MAX_SPEED (200.0)
#define THREE_QUARTER_SPEED (150.0)
#define HALF_SPEED (100.0)
#define NINETY_SPEED (90.0)
#define SEVENTY_FIVE_SPEED (75.0)
#define SIXTY_SPEED (60.0)
#define FIFTY_SPEED (50.0)
#define FORTY_SPEED (40.0)
#define TWENTY_FIVE_SPEED (25.0)

//ARM PLACE POSITIONS
#define NUMBER_OF_HEIGHTS 5
#define ARM_POSITION_BOTTOM 0
#define ARM_DEPLOY 240
#define ALLIANCE_TOWER 440
#define SHORT_TOWER  480
#define MEDIUM_HIGH_TOWER 6000

//TRAY POSITIONS
#define TRAY_BOTTOM_POS 0
#define TRAY_ARM_POS 360
#define TRAY_PLACE_POS 360
#define TRAY_SLOW_POS 160

const int POSITIONS[NUMBER_OF_HEIGHTS] =
  {ARM_POSITION_BOTTOM, ALLIANCE_TOWER, SHORT_TOWER, MEDIUM_HIGH_TOWER, ARM_DEPLOY};

extern pros::Motor a_mtr;
extern pros::Motor t_mtr;
extern pros::Motor li_mtr;
extern pros::Motor ri_mtr;
extern double get_proper_gyro();
extern void gyro_turn(double angle);

//TRAY PID
auto tray = AsyncControllerFactory::posIntegrated(TRAY_MOTOR_PORT);

void tray_return()
{
  tray.setMaxVelocity(MAX_SPEED);
  tray.setTarget(TRAY_BOTTOM_POS);
  tray.waitUntilSettled();
}

void tray_up()
{
  tray.setMaxVelocity(MAX_SPEED);
  tray.setTarget(TRAY_ARM_POS);
  tray.waitUntilSettled();
}

void three_cubes()
{
  tray.reset();
  tray.setMaxVelocity(FORTY_SPEED);
  tray.setTarget(TRAY_PLACE_POS);
  tray.waitUntilSettled();

}

void six_cubes()
{
  tray.reset();
  tray.setMaxVelocity(FORTY_SPEED);
  tray.setTarget(TRAY_PLACE_POS);
  tray.waitUntilSettled();
}

void nine_cubes()
{
  tray.reset();
  tray.setMaxVelocity(FORTY_SPEED);
  tray.setTarget(TRAY_PLACE_POS);
  tray.waitUntilSettled();
}

//ARM POSITION PID
auto arm = AsyncControllerFactory::posIntegrated(ARM_MOTOR_PORT);

void bottom()
{
  arm.setTarget(POSITIONS[0]);
  arm.waitUntilSettled();
}

void alliance_tower()
{
  arm.setTarget(POSITIONS[1]);
  arm.waitUntilSettled();
}

void short_tower()
{
  arm.setTarget(POSITIONS[2]);
  arm.waitUntilSettled();
}

void medium_high_tower()
{
  arm.setTarget(POSITIONS[3]);
  arm.waitUntilSettled();
}

void deploy()
{
  arm.setTarget(POSITIONS[4]);
  arm.waitUntilSettled();
}

//CHASSIS PID
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

//INTAKE VELOCITY PID
auto intakes = AsyncControllerFactory::velIntegrated
(
  {LEFT_INTAKE_MOTOR_PORT, RIGHT_INTAKE_MOTOR_PORT}
);

void intake_on(double speed = 200.0)
{
  intakes.setTarget(speed);
  intakes.waitUntilSettled();
}

void intake_off()
{
  intakes.setTarget(0);
  //intakes.reset();
  intakes.waitUntilSettled();
}

void autonomous()
{

  pros::Motor lf_mtr(LEFT_FRONT_WHEEL_PORT);
  pros::Motor lr_mtr(LEFT_REAR_WHEEL_PORT);
  pros::Motor rf_mtr(RIGHT_FRONT_WHEEL_PORT, true);
  pros::Motor rr_mtr(RIGHT_REAR_WHEEL_PORT, true);
  pros::Motor a_mtr(ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
  pros::Motor t_mtr(TRAY_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
  pros::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT);
  pros::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT, true);

  a_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  t_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  li_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ri_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lr_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lf_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

//TEST

  tray_up();
  tray_return();
  deploy();
  bottom();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.moveDistanceAsync(-1_in);
  chassis.waitUntilSettled();
  chassis.moveDistanceAsync(42_in);
  intake_on(200);
  chassis.waitUntilSettled();
  intake_off();
  gyro_turn(-15);

  //gyro_turn(90);
  //tray_up();
  //short_tower();
  //intake_on(-100);
  //pros::delay(1500);
  //intake_off();
  //chassis.setMaxVelocity(HALF_SPEED);
  //chassis.moveDistanceAsync(7_in);
  //intake_on(200);
  //pros::delay(5000);
  //chassis.waitUntilSettled();
  //intake_off();
  //short_tower();
  //six_cubes();
  //tray_return();

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


/*
//FRONT
    chassis.setMaxVelocity(MAXER_SPEED);
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
    chassis.moveDistanceAsync(-11_in);
    chassis.waitUntilSettled();
    chassis.moveDistanceAsync(11_in);
    chassis.waitUntilSettled();
*/

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
}
