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

//ARM PLACE POSITIONS
#define NUMBER_OF_HEIGHTS 5
#define ARM_POSITION_BOTTOM 0
#define ARM_DEPLOY 240
#define ALLIANCE_TOWER 460
#define SHORT_TOWER  480
#define MEDIUM_HIGH_TOWER 600

//TRAY POSITIONS
#define TRAY_BOTTOM_POS -30
#define TRAY_ARM_POS 360
#define TRAY_PLACE_POS 720
#define TRAY_SLOW_POS 160
#define TRAY_TEST_POS 650

const int POSITIONS[NUMBER_OF_HEIGHTS] =
  {ARM_POSITION_BOTTOM, ALLIANCE_TOWER, SHORT_TOWER, MEDIUM_HIGH_TOWER, ARM_DEPLOY};

//TRAY PID
auto tray = AsyncControllerFactory::posIntegrated(TRAY_MOTOR_PORT);

//Different tray positions
void tray_return()
{
  tray.setMaxVelocity(NINETY_SPEED);
  tray.setTarget(TRAY_BOTTOM_POS);
  tray.waitUntilSettled();
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
  tray.setMaxVelocity(SIXTY_SPEED);
  tray.setTarget(TRAY_TEST_POS);
  tray.waitUntilSettled();
}

void six_cubes()
{
  tray.reset();
  tray.setMaxVelocity(SIXTY_SPEED);
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

void forty_five_deg_turn(QAngle angle = 45_deg)
{
  gyro_turn(chassis, angle, 100, 20, 0.014, 0.0, 0.0, 2);
}

void fifty_deg_to_eighty_deg_turn()
{
  gyro_turn(chassis, 45_deg, 100, 22.5);
}

void ninety_deg_turn()
{
  gyro_turn(chassis, 90_deg, 100, 17.5, 0.011, 0.0, 0.0, 2.5);
}

void one_hundred_eighty_deg_turn()
{
  gyro_turn(chassis, 180_deg, 100, 17.5, 0.011, 0.0, 0.0, 2);
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
  tray_return();
  async_gyro_drive(chassis, -10_in, 40);
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

void test_nine_point();
void blue_back_port_5();
void red_back_port_4();
void red_front_port_3();
void blue_front_port_2();
void skills_port_1();
void testing();
void testing_nine_same_stack();
void testing_nine_diff_stack(double start_time_x);
void new_blue_back_port_5();
void new_red_back_port_5();
void testing_new();

void autonomous()
{

  pros::lcd::print(0, "autonomous");

  //Break modes
  double start_time = pros::c::millis();
  a_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  t_mtr.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  li_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  ri_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
  //chassis.setBrakeMode(AbstractMotor::brakeMode::brake);
  chassis.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  //Resets the gyro so that changes to the position during pre-autonomous do not affect autonomous
  //gyro_reset();

  testing_new();

  //testing_nine_diff_stack(start_time);
  //blue_front_port_2();
  //new_blue_back_port_5();

  double end_time = pros::c::millis();
  pros::lcd::print(6,"autonomous Time %f", end_time - start_time);
}

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
