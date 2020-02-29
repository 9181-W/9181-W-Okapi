#include "main.h"
#include "okapi/api.hpp"
#include "gyro.h"
#include "autos.h"
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
#define LEFT_FRONT_WHEEL_PORT 13
#define LEFT_REAR_WHEEL_PORT 11
#define RIGHT_FRONT_WHEEL_PORT 10
#define RIGHT_REAR_WHEEL_PORT 20
#define TRAY_MOTOR_PORT 15
#define ARM_MOTOR_PORT 16
#define LEFT_INTAKE_MOTOR_PORT 1
#define RIGHT_INTAKE_MOTOR_PORT 6

#define MAX_SPEED (200.0)
#define THREE_QUARTER_SPEED (150.0)
#define HALF_SPEED (100.0)
#define NINETY_SPEED (90.0)
#define SEVENTY_FIVE_SPEED (75.0)
#define SIXTY_SPEED (60.0)
#define FIFTY_SPEED (50.0)
#define FORTY_SPEED (40.0)
#define TWENTY_FIVE_SPEED (25.0)
#define TEN_SPEED (10.0)

//ARM PLACE POSITIONS
#define NUMBER_OF_HEIGHTS 5
#define ARM_POSITION_BOTTOM 0
#define ARM_DEPLOY 240
#define ALLIANCE_TOWER 460
#define SHORT_TOWER  480
#define MEDIUM_HIGH_TOWER 600

//TRAY POSITIONS
#define TRAY_BOTTOM_POS -30
#define TRAY_ARM_POS 260
#define TRAY_PLACE_POS 720
#define TRAY_SLOW_POS 160
#define TRAY_TEST_POS 710

const int POSITIONS[NUMBER_OF_HEIGHTS] =
  {ARM_POSITION_BOTTOM, ALLIANCE_TOWER, SHORT_TOWER, MEDIUM_HIGH_TOWER, ARM_DEPLOY};

//TRAY PID
auto tray = AsyncControllerFactory::posIntegrated(TRAY_MOTOR_PORT);

//Different tray positions
void tray_return()
{
  tray.setMaxVelocity(TWENTY_FIVE_SPEED);
  tray.setTarget(TRAY_BOTTOM_POS);
}

void tray_half()
{
  tray.setMaxVelocity(MAX_SPEED);
  tray.setTarget(TRAY_ARM_POS);
}

void tray_return_fast()
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
  tray.setMaxVelocity(MAX_SPEED);
  tray.setTarget(TRAY_PLACE_POS);
  tray.waitUntilSettled();
}

void six_cubes()
{
  tray.reset();
  tray.setMaxVelocity(FORTY_SPEED);
  tray.setTarget(TRAY_TEST_POS);
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

//Different arm positions
void bottom()
{
  arm.setMaxVelocity(100);
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

void forty_five_deg_turn(QAngle angle)
{
  //gyro_turn(chassis, angle, 100, 20, 0.014, 0.0, 0.0, 2);
  gyro_turn(chassis, angle, 100, 25, 0.014, 0.0, 0.0, 3);
}

void fifty_deg_turn(QAngle angle)
{
  //gyro_turn(chassis, angle, 100, 20, 0.014, 0.0, 0.0, 2);
  gyro_turn(chassis, angle, 100, 25, 0.010, 0.0, 0.0, 3);

}


void ninety_deg_turn(QAngle new_angle)
{
  //gyro_turn(chassis, new_angle, 100, 17.5, 0.00090, 0.0, 0.0, 2.5);
  gyro_turn(chassis, new_angle, 100, 17.5, 0.0090, 0.0, 0.0, 2.5);
}

void ninety_deg_turn_heavy_inaccurate(QAngle new_angle)
{
  //gyro_turn(chassis, new_angle, 100, 50, 0.010, 0.0, 0.0, 10);
  gyro_turn(chassis, new_angle, 100, 50, 0.0090, 0.0, 0.0, 10);
}

void one_hundred_twenty_deg_turn_heavy(QAngle new_angle)
{
  gyro_turn(chassis, new_angle, 80, 22.5, 0.05, 0.0, -0.25, 2);
}

void one_hundred_eighty_deg_turn()
{
  gyro_turn(chassis, 180_deg, 100, 17.5, 0.00080, 0.0, 0.0, 2);
}


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

void intake_off()
{
  intakes.setTarget(0);
  //intakes.reset();
  //intakes.waitUntilSettled();
}

void nine_cube_place()
{
  intake_on(100);
  nine_cubes();
  intake_off();
  pros::delay(500);
  tray_return();
  async_gyro_drive(chassis, -10_in, 20);
  intake_on(-40);
  wait_for_drive_complete();
  intake_off();
}

void edited_nine_cube_place()
{
  intake_on(100);
  nine_cubes();
  intake_off();
  pros::delay(200);
  tray_return_fast();
  async_gyro_drive(chassis, -16_in, 40);
  intake_on(-65);
  wait_for_drive_complete();
  intake_off();
}

void edited_seven_cube_place()
{
  intake_on(-80);
  pros::delay(575);
  intake_off();
  intake_on(19);
  six_cubes();
  intake_off();
  pros::delay(500);
  async_gyro_drive(chassis, -7.75_in, 40);
  intake_on(-50);
  tray_return_fast();
  wait_for_drive_complete();
  intake_off();
}

void seven_cube_place()
{
  intake_on(18.33);
  three_cubes();
  intake_off();
  pros::delay(500);
  tray_return_fast();
  async_gyro_drive(chassis, -7.75_in, 40);
  intake_on(-60);
  wait_for_drive_complete();
  intake_off();
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

void init_auto()
{
  a_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  li_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  ri_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

const double arm_epsilon = 2.0;
const double arm_kp = 0.8;

double arm_error = 0.0;

void arm_pos_P(double arm_target_position, double arm_max_speed)
{
  double arm_start_position = a_mtr.getPosition();
  double arm_current_position = a_mtr.getPosition();
  arm_error = arm_target_position - arm_current_position;

  while(arm_error > arm_epsilon) {

    double arm_speed = arm_error * arm_kp;

    a_mtr.moveVelocity(arm_speed);

    //Removes impossible speeds by setting the speed down to a possible one
    if(arm_speed > arm_max_speed) {
      arm_speed = arm_max_speed;
    }

    if(arm_speed < arm_max_speed * -1) {
      arm_speed = arm_max_speed * -1;
    }

    arm_current_position = a_mtr.getPosition();

    arm_error = arm_target_position - arm_current_position;

    pros::delay(33);
  }
  a_mtr.move(0);
}

const double tray_epsilon = 10.0;
const double tray_kp = 0.8;

double tray_distance_remaining = 0.0;

void tray_stack(double tray_target_position, double tray_max_speed)
{
  //Resets the position of the encoder to zero
  //t_mtr.tarePosition();

  double tray_start_position = t_mtr.getPosition();
  double tray_current_position = t_mtr.getPosition();
  tray_distance_remaining = tray_target_position - tray_current_position;

  while(tray_distance_remaining > tray_epsilon)
  {

      double tray_speed = tray_distance_remaining; //* tray_kp;

      t_mtr.moveVelocity(tray_speed);

      //Removes impossible speeds by setting the speed down to a possible one
      if(tray_speed > tray_max_speed)
      {
          tray_speed = tray_max_speed;
      }

      if(tray_speed < tray_max_speed * -1)
      {
          tray_speed = tray_max_speed * -1;
      }

      tray_current_position = t_mtr.getPosition();

      tray_distance_remaining = tray_target_position - tray_current_position;

      pros::delay(33);
  }
  t_mtr.move(0);
}
