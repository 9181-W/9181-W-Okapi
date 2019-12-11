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
#define RIGHT_INTAKE_MOTOR_PORT 5

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
#define TRAY_PLACE_POS 440
#define TRAY_SLOW_POS 160

const int POSITIONS[NUMBER_OF_HEIGHTS] =
  {ARM_POSITION_BOTTOM, ALLIANCE_TOWER, SHORT_TOWER, MEDIUM_HIGH_TOWER, ARM_DEPLOY};

extern double get_proper_gyro();
extern void gyro_turn(QAngle q_angle);
extern void gyro_turn_to(QAngle q_angle);

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
  tray.setMaxVelocity(TWENTY_FIVE_SPEED);
  tray.setTarget(TRAY_PLACE_POS);
  tray.waitUntilSettled();
}

//ARM POSITION PID
auto arm = AsyncControllerFactory::posIntegrated(ARM_MOTOR_PORT);

void bottom()
{
  arm.setTarget(POSITIONS[0]);
  //arm.waitUntilSettled();
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
okapi::ChassisControllerIntegrated chassis = ChassisControllerFactory::create
(
  {LEFT_FRONT_WHEEL_PORT, LEFT_REAR_WHEEL_PORT},
  {-RIGHT_FRONT_WHEEL_PORT, -RIGHT_REAR_WHEEL_PORT},
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
  //intakes.waitUntilSettled();
}

void intake_on_slow(double speed = 150.0)
{
  intakes.setTarget(speed);
  //intakes.waitUntilSettled();
}

void intake_off()
{
  intakes.setTarget(0);
  //intakes.reset();
  //intakes.waitUntilSettled();
}



okapi::Motor a_mtr(ARM_MOTOR_PORT,true,AbstractMotor::gearset::red);
okapi::Motor t_mtr(TRAY_MOTOR_PORT,false,AbstractMotor::gearset::red);
okapi::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT,false,AbstractMotor::gearset::green);
okapi::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT,true,AbstractMotor::gearset::green);
okapi::Motor lf_mtr(LEFT_FRONT_WHEEL_PORT,false,AbstractMotor::gearset::green);
okapi::Motor lr_mtr(LEFT_REAR_WHEEL_PORT,false,AbstractMotor::gearset::green);
okapi::Motor rf_mtr(RIGHT_FRONT_WHEEL_PORT,true,AbstractMotor::gearset::green);
okapi::Motor rr_mtr(RIGHT_REAR_WHEEL_PORT,true,AbstractMotor::gearset::green);

void autonomous()
{
  double start_time = pros::c::millis();
  a_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  li_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  ri_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setBrakeMode(AbstractMotor::brakeMode::brake);

//TEST

intake_on(-85);
pros::delay(1000);
intake_off();
gyro_turn_to(0_deg);
chassis.setMaxVelocity(85);
intake_on();
chassis.moveDistanceAsync(40_in);
chassis.waitUntilSettled();
pros::delay(400);
intake_off();
//gyro_turn_to(0_deg);
chassis.moveDistanceAsync(6_in);
intake_on();
chassis.waitUntilSettled();
intake_off();
intake_on();
pros::delay(400);
intake_off();
chassis.moveDistanceAsync(-7_in);
chassis.waitUntilSettled();
gyro_turn_to(0_deg);
gyro_turn_to(25_deg);
chassis.moveDistanceAsync(11_in);
intake_on();
chassis.waitUntilSettled();
intake_off();
chassis.moveDistanceAsync(-3_in);
chassis.waitUntilSettled();
gyro_turn_to(110_deg);
chassis.moveDistanceAsync(32_in);
intake_on();
chassis.waitUntilSettled();
intake_off();
chassis.moveDistanceAsync(6_in);
nine_cubes();

//SMALL ZONE BLUE --SLOT 1
/*
chassis.setMaxVelocity(200);
//chassis.moveDistanceAsync(6_in);
//chassis.waitUntilSettled();
//chassis.moveDistanceAsync(-6_in);
//chassis.waitUntilSettled();
deploy();
bottom();
//gyro_turn_to(0_deg);
chassis.setMaxVelocity(100);
intake_on();
chassis.moveDistanceAsync(44_in);
chassis.waitUntilSettled();
intake_off();
chassis.moveDistanceAsync(-20_in);
chassis.waitUntilSettled();
gyro_turn_to(-131_deg);
chassis.setMaxVelocity(75);
chassis.moveDistanceAsync(16_in);
chassis.waitUntilSettled();
intake_on_slow(-90);
pros::delay(200);
intake_off();
nine_cubes();
chassis.moveDistanceAsync(-12_in);
chassis.waitUntilSettled();
*/

//SMALL ZONE RED --SLOT 2

/*
chassis.setMaxVelocity(200);
//chassis.moveDistanceAsync(6_in);
//chassis.waitUntilSettled();
//chassis.moveDistanceAsync(-6_in);
//chassis.waitUntilSettled();
deploy();
bottom();
//gyro_turn_to(0_deg);
chassis.setMaxVelocity(100);
intake_on();
chassis.moveDistanceAsync(44_in);
chassis.waitUntilSettled();
intake_off();
chassis.moveDistanceAsync(-26_in);
chassis.waitUntilSettled();
gyro_turn_to(129_deg);
chassis.setMaxVelocity(75);
chassis.moveDistanceAsync(14_in);
chassis.waitUntilSettled();\
intake_on_slow(-90);
pros::delay(200);
intake_off();
nine_cubes();
chassis.moveDistanceAsync(-12_in);
chassis.waitUntilSettled();
*/

//LARGE ZONE BLUE --SLOT 3

/*
chassis.setMaxVelocity(200);
//chassis.moveDistanceAsync(6_in);
//chassis.waitUntilSettled();
//chassis.moveDistanceAsync(-6_in);
//chassis.waitUntilSettled();
deploy();
bottom();
//chassis.moveDistanceAsync(-1_in);
//chassis.waitUntilSettled();
gyro_turn_to(0_deg);
chassis.setMaxVelocity(115);
intake_on();
chassis.moveDistanceAsync(46_in);
chassis.waitUntilSettled();
pros::delay(400);
intake_off();
gyro_turn_to(132_deg);
chassis.setMaxVelocity(100);
intake_on_slow();
chassis.moveDistanceAsync(40.5_in);
chassis.waitUntilSettled();
intake_off();
intake_on_slow(-90);
pros::delay(400);
intake_off();
nine_cubes();
chassis.setMaxVelocity(50);
chassis.moveDistanceAsync(2_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(-12_in);
chassis.waitUntilSettled();
*/

//LARGE ZONE RED --SLOT 4

/*
chassis.setMaxVelocity(200);
//chassis.moveDistanceAsync(6_in);
//chassis.waitUntilSettled();
//chassis.moveDistanceAsync(-6_in);
//chassis.waitUntilSettled();
deploy();
bottom();
//chassis.moveDistanceAsync(-1_in);
//chassis.waitUntilSettled();
pros::delay(500);
gyro_turn_to(0_deg);
chassis.setMaxVelocity(115);
intake_on();
chassis.moveDistanceAsync(46_in);
chassis.waitUntilSettled();
pros::delay(400);
intake_off();
gyro_turn_to(-132_deg);
chassis.setMaxVelocity(100);
intake_on_slow();
chassis.moveDistanceAsync(39.5_in);
chassis.waitUntilSettled();
intake_off();
intake_on_slow(-90);
pros::delay(400);
intake_off();
nine_cubes();
chassis.setMaxVelocity(25);
chassis.moveDistanceAsync(2_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(-12_in);
chassis.waitUntilSettled();
*/












/*
chassis.setMaxVelocity(200);
chassis.moveDistanceAsync(6_in);  // Maybe these could be 4 inches.... test at school.
chassis.waitUntilSettled();
chassis.moveDistanceAsync(-6_in);
chassis.waitUntilSettled();
deploy();
bottom();
gyro_turn_to(0_deg);  // This could be removed if you trust driving into wall... maybe doubtful.
chassis.setMaxVelocity(100);
intake_on();  // You can remove wait for settled in intake_on function
chassis.moveDistanceAsync(40_in);
chassis.waitUntilSettled();
intake_off();   // You can remove wait for settled in intake_off function
chassis.setMaxVelocity(200);
gyro_turn_to(-28_deg);
intake_on();
chassis.moveDistanceAsync(8_in);
chassis.waitUntilSettled();
intake_off();
chassis.setMaxVelocity(200);
gyro_turn_to(10_deg);
chassis.setMaxVelocity(200);  // Might as well drive full speed backwards
chassis.moveDistanceAsync(-32_in);
chassis.waitUntilSettled();
chassis.setMaxVelocity(200);
gyro_turn_to(140_deg);
chassis.moveDistanceAsync(14_in);  // You can eliminate this drive by changing -10 and drive -30.
chassis.waitUntilSettled();
six_cubes();
chassis.moveDistanceAsync(-10_in);
chassis.waitUntilSettled();
//pros::delay(5000);
//gyro_turn(180_deg);
//chassis.moveDistanceAsync(-20_in);
//chassis.waitUntilSettled();
*/

/*
chassis.setMaxVelocity(75);
chassis.moveDistanceAsync(6_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(-6_in);
chassis.waitUntilSettled();
deploy();
bottom();
chassis.setMaxVelocity(100);
chassis.moveDistanceAsync(-3.25_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(42_in);
intake_on(170);
chassis.waitUntilSettled();
intake_off();
//gyro_turn(28_deg);
//chassis.moveDistanceAsync(7_in);
//intake_on(170);
//chassis.waitUntilSettled();
gyro_turn(168_deg);
//intake_on(170);
chassis.moveDistance(34_in);
//intake_off();
nine_cubes();
chassis.moveDistance(-6_in);
*/

/*
chassis.setMaxVelocity(75);
chassis.moveDistanceAsync(6_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(-6_in);
chassis.waitUntilSettled();
deploy();
bottom();
chassis.setMaxVelocity(100);
chassis.moveDistanceAsync(-2.5_in);
chassis.waitUntilSettled();
chassis.moveDistanceAsync(42_in);
intake_on(170);
chassis.waitUntilSettled();
intake_off();
//gyro_turn(28_deg);
//chassis.moveDistanceAsync(7_in);
//intake_on(170);
//chassis.waitUntilSettled();
gyro_turn(-154.5_deg);
//intake_on(170);
chassis.moveDistance(31_in);
//intake_off();
nine_cubes();
chassis.moveDistance(-6_in);
*/

/*
chassis.moveDistance(9_in);
chassis.moveDistance(-9_in);
*/

  double end_time = pros::c::millis();
  pros::lcd::print(0,"autonomous Time %f",end_time - start_time);

  while(true)
  {
    pros::lcd::print(7,"Gryo Value %f",get_proper_gyro());
    pros::delay(10);
  }
//gyro_turn(-90_deg);
//chassis.waitUntilSettled();
//chassis.moveDistanceAsync(9_in);
}
