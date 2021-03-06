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
#define LEFT_ARM_MOTOR_PORT 15
#define RIGHT_ARM_MOTOR_PORT 16
#define LEFT_INTAKE_MOTOR_PORT 8
#define RIGHT_INTAKE_MOTOR_PORT 2

#define MEGA_MAX_SPEED (100.0)
#define MAX_SPEED (80.0)
#define THREE_QUARTER_SPEED (60.0)
#define HALF_SPEED (40.0)

#define LIFT_POSITION_BOTTOM 0.0
#define LIFT_POSITION_MIDDLE 2000.0
#define LIFT_POSITION_TOP    4000.0
/*
pros::Motor la_mtr(LEFT_ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
pros::Motor ra_mtr(RIGHT_ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true);
pros::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT);
pros::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT, true);

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
*/
void autonomous()
{
  // Create controllers
  auto chassis = ChassisControllerFactory::create(
    {LEFT_FRONT_WHEEL_PORT, LEFT_REAR_WHEEL_PORT},
    {-RIGHT_FRONT_WHEEL_PORT, -RIGHT_REAR_WHEEL_PORT},
    IterativePosPIDController::Gains{0.005, 0, 0.0001},
    IterativePosPIDController::Gains{0.005, 0, 0.0001},
    IterativePosPIDController::Gains{0.005, 0, 0.0001},
    AbstractMotor::gearset::green,
    {4.0_in, 14.375_in}
  );

  pros::Motor la_mtr(LEFT_ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36);
  pros::Motor ra_mtr(RIGHT_ARM_MOTOR_PORT, pros::E_MOTOR_GEARSET_36, true);
  pros::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT);
  pros::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT, true);

  la_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ra_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  li_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  ri_mtr.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

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

// Skills
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
  li_mtr.move(200);
  ri_mtr.move(200);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(8.5_in);
  li_mtr.move(160);
  ri_mtr.move(160);
  chassis.waitUntilSettled();
  li_mtr.move(-75);
  ri_mtr.move(-75);
  pros::delay(100);
  li_mtr.move(100);
  ri_mtr.move(100);
  pros::delay(100);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(40_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(-10.4_in);
  li_mtr.move(200);
  ri_mtr.move(200);
  chassis.waitUntilSettled();
  li_mtr.move(200);
  ri_mtr.move(200);
  pros::delay(200);
  li_mtr.move(0);
  ri_mtr.move(0);
  pros::delay(400);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-48_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(13_in);
  li_mtr.move(150);
  ri_mtr.move(150);
  chassis.waitUntilSettled();
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
  chassis.moveDistanceAsync(-9_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-173_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  li_mtr.move(180);
  ri_mtr.move(180);
  pros::delay(1600);
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(15_in);
  chassis.waitUntilSettled();
  chassis.setMaxVelocity(HALF_SPEED);
  chassis.turnAngle(-103_deg);
  chassis.setMaxVelocity(MAX_SPEED);
  chassis.moveDistanceAsync(19_in);
  li_mtr.move(200);
  ri_mtr.move(200);
  chassis.waitUntilSettled();
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
  */

  //Back Blue
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
  pros::delay(200);
  chassis.moveDistanceAsync(41_in);
  li_mtr.move(160);
  ri_mtr.move(160);
  chassis.waitUntilSettled();
  li_mtr.move(0);
  ri_mtr.move(0);
  chassis.setMaxVelocity(THREE_QUARTER_SPEED);
  chassis.turnAngle(35_deg);
  chassis.setMaxVelocity(MAX_SPEED);
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
  chassis.turnAngle(185_deg);
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



}
