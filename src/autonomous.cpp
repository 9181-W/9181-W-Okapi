#include "main.h"
#include "okapi/api.hpp"
#include "gyro.h"
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


//Deinfes different wire ports and positions
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
#define MEDIUM_HIGH_TOWER 550

//TRAY POSITIONS
#define TRAY_BOTTOM_POS 0
#define TRAY_ARM_POS 360
#define TRAY_PLACE_POS 425
#define TRAY_SLOW_POS 160

const int POSITIONS[NUMBER_OF_HEIGHTS] =
  {ARM_POSITION_BOTTOM, ALLIANCE_TOWER, SHORT_TOWER, MEDIUM_HIGH_TOWER, ARM_DEPLOY};

//TRAY PID
auto tray = AsyncControllerFactory::posIntegrated(TRAY_MOTOR_PORT);

//Different tray positions
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

//Different arm positions
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

//Different intake speeds
void intake_on(double speed = 180.0)
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

//Creates Motors
okapi::Motor a_mtr(ARM_MOTOR_PORT,true,AbstractMotor::gearset::red);
okapi::Motor t_mtr(TRAY_MOTOR_PORT,true,AbstractMotor::gearset::red);
okapi::Motor li_mtr(LEFT_INTAKE_MOTOR_PORT,false,AbstractMotor::gearset::green);
okapi::Motor ri_mtr(RIGHT_INTAKE_MOTOR_PORT,true,AbstractMotor::gearset::green);
okapi::Motor lf_mtr(LEFT_FRONT_WHEEL_PORT,false,AbstractMotor::gearset::green);
okapi::Motor lr_mtr(LEFT_REAR_WHEEL_PORT,false,AbstractMotor::gearset::green);
okapi::Motor rf_mtr(RIGHT_FRONT_WHEEL_PORT,true,AbstractMotor::gearset::green);
okapi::Motor rr_mtr(RIGHT_REAR_WHEEL_PORT,true,AbstractMotor::gearset::green);

void autonomous()
{

  pros::lcd::print(0, "autonomous");

  //Break modes
  double start_time = pros::c::millis();
  a_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  li_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  ri_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  //Resets the gyro so that changes to the position during pre-autonomous do not affect autonomous
  gyro_reset();

  //async_gyro_drive(chassis, 48_in, 50);
  gyro_turn(chassis, 90_deg, 100);

  double end_time = pros::c::millis();
  pros::lcd::print(1,"autonomous Time %f",end_time - start_time);
}
