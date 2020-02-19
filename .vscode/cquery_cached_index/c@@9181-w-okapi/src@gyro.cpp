#include "main.h"
#include "okapi/api.hpp"
#include "gyro.h"
#include "inertial.h"
using namespace okapi;

#define ADI_GYRO_PORT 2

ADIGyro* gyro_B = NULL;


//Converts the gyro value from raw units to degrees
//then scales the gyro so that a full turn = 360 degrees
double get_proper_gyro()
{
  return get_inertial_value();
}

void gyro_reset()
{
  inertial_reset();
}


//Task code that displays the proper gyro value on line 7 of the lcd
void gyro_display(void* param)
{
  while (true)
  {
    //pros::lcd::print(7,"Gryo Value %f",get_proper_gyro());
    pros::delay(50);
  }
}


//Initializes the gyro and starts the task
void gyro_initialize()
{
  //Will stop all action until gyro is initialized (1.3 seconds)
  //DO NOT MOVE ROBOT DURING THIS TIME
  gyro_B = new ADIGyro(ADI_GYRO_PORT);

  pros::Task gyro_display_task (gyro_display, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Gyro Display Task");

}


//Creates a constant for wheel diameter
const double wheel_diam = 4.0;
//Creates a constant for pi
const double drive_pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * drive_pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;

//Drive X distance at Y speed
//void drive(double distance_in_inches, double max_speed)
void gyro_drive(okapi::ChassisController& chassis, QLength distance, double max_speed, bool drive_straight)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    max_speed = max_speed / 100;
    //Sets the encoder units to use degrees instead of ticks
    chassis.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

    /*//Setting the Proportional,Integral,Differential constants (P.I.D.)
    const double drive_kp = 0.00158;
    const double drive_ki = 0.0002;
    const double drive_kd = 0.0005;
    */

    const double drive_kp = 0.0018;
    const double drive_ki = 0.00008;
    const double drive_kd = 0.0001;
    //Creates a constant for allowable error before stopping
    const double epsilon = 2.0;
    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double maximum_vel_adj = 0.1;
    //States the window in which the integral will be activated
    const double integral_limit = 250.0;
    //Sets the proportional constant for driving straight
    const double drive_straight_kp = 0.05;

    //Converts Qlength distance to distance_in_inches
    double distance_in_inches = distance.convert(inch);
    //Convert distance to encoder degrees
    double distance_in_degrees = distance_in_inches * degrees_per_inch;
    //Draws starting position from the encoders (found in chassisController.cpp on github)
    std::valarray<std::int32_t> start_pos_values = chassis.getSensorVals();
    //Calculates current position based on start position (found in chassisController.cpp on github)
    std::valarray<std::int32_t> current_pos_values = chassis.getSensorVals() - start_pos_values;
    //Sets the integral to zero so that additions can be made latetr
    double integral = 0.0;
    //Sets last error to zero before driving starts
    double last_error = 0.0;

    double second_last_error = 9999.9;

    double third_last_error = 9999.9;

    double fourth_last_error = 9999.9;
    //Defines the initial drive error (found in chassisController.cpp on github)
    double drive_error = distance_in_degrees - static_cast<double>((current_pos_values[0] + current_pos_values[1])) / 2.0;
    //Creates a variable that contains the initial gyro value (0)
    gyro_reset();
    double initial_drive_gyro_value = get_proper_gyro();
    //Sets the first speed to zero
    double last_speed = 0.0;

    double last_three_derivatives = 9999.9;



    printf("distance: %f  error: %f\n",distance_in_degrees,drive_error);

    //Drive while the robot hasn't reached its target distance
    while(fabs(last_three_derivatives) > epsilon)
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        //Calculate distance left to drive
        drive_error = distance_in_degrees - static_cast<double>((current_pos_values[0] + current_pos_values[1])) / 2.0;;

        printf("distance: %f  error: %f ",distance_in_degrees,drive_error);

        //Calculates the derivative
        double derivative = last_error - drive_error;

        fourth_last_error = third_last_error;

        third_last_error = second_last_error;

        second_last_error = last_error;

        //Sets a new last error
        last_error = drive_error;

        double third_last_derivative = fourth_last_error - third_last_error;

        double second_last_derivative = third_last_error - second_last_error;

        double last_derivative = second_last_error - last_error;

        last_three_derivatives = last_derivative + second_last_derivative + third_last_derivative;

        //Determines when the integral will start being used
        if(drive_ki != 0)
        {
            if(fabs(drive_error) < integral_limit)
            {
                integral = integral + drive_error;
            }
            else
            {
                integral = 0;
            }
        }

        if((integral > 0) && (drive_error < 0))
        {
          integral = integral * -1;
        }
        else if((integral < 0) && (drive_error > 0))
        {
          integral = integral * -1;
        }

        //Calculate speed to be driven at using kp,ki,kd
        double speed = drive_error * drive_kp + integral * drive_ki + derivative * drive_kd;
        printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);

        //Removes impossible speeds by setting the speed down to a possible one
        if(speed > max_speed)
        {
            speed = max_speed;
        }

        if(speed < max_speed * -1)
        {
            speed = max_speed * -1;
        }


        // ************************************************************************************************* //
        // Set maximum accelleration to prevent the robot from jerking right or left at the start of driving //
        // ************************************************************************************************* //

        double velocity_adj = speed - last_speed;

        if(velocity_adj > maximum_vel_adj)
        {
            speed = last_speed + maximum_vel_adj;
        }

         if(velocity_adj < -maximum_vel_adj)
        {
            speed = last_speed + -maximum_vel_adj;
        }

        last_speed = speed;
        printf("adj speed: %f\n",speed);


        // ****************************************************************************************************************************** //
        // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
        // - This does not support driving backwards right now.                                                                           //
        // ****************************************************************************************************************************** //
        double turn_speed = 0.0;
        if(drive_straight == true)
        {
          //Gets the gyro's current value
          double drive_gyro_value = get_proper_gyro();

          //Calculates the amount that the robot is off of its heading
          double drive_straight_error = initial_drive_gyro_value - drive_gyro_value;

          //Creates a turn speed so that different sides can be slowed down
          turn_speed = drive_straight_error * drive_straight_kp;
        }


        // ******************************************* //
        // Set final speed and calculate the new error //
        // ******************************************* //

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        chassis.arcade(speed, turn_speed);
        pros::delay(33);

        //Calculates current position based on start position after small movement
        current_pos_values = chassis.getSensorVals() - start_pos_values;

    }

    //Stops the robot from moving after the robot has reached its target distance
    chassis.stop();
}

okapi::ChassisController* async_chassis;
QLength async_distance;
double async_max_speed;
bool async_complete = true;
pros::Task* drive_task = NULL;

void drive_async(void* param)
{
  while (true)
  {
    if(!async_complete)
    {
      gyro_drive(*async_chassis, async_distance, async_max_speed);
      async_complete = true;
    }
    pros::delay(33);
  }
}

bool drive_is_complete()
{
  return async_complete;
}

void wait_for_drive_complete()
{
  while(!async_complete)
  {
    pros::delay(10);
  }
}

void async_gyro_drive(okapi::ChassisController& chassis, QLength distance, double max_speed)
{
  async_chassis = &chassis;
  async_distance = distance;
  async_max_speed = max_speed;

  if (drive_task == NULL)
  {
    drive_task = new pros::Task(drive_async, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Drive Task");
  }

  async_complete = false;
}

//Turn x degrees at y speed
void gyro_turn(okapi::ChassisController& chassis, QAngle angle, double max_speed, double min_speed, double kp, double ki, double kd, double epsilon)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    //if (max_speed > 60) max_speed = 60;
    max_speed = max_speed / 100;
    if (min_speed < 17.5) min_speed = 17.5;
    min_speed = min_speed / 100;

    //Sets the encoder units to se degrees instead of ticks
    chassis.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    //Setting the Proportional,Integral,Differential constants (P.I.D.)
    double turn_kp = kp;//0.011;
    double turn_ki = ki;//0.000;
    double turn_kd = kd;//0.000;
    //Creates a constant for allowable error before stopping
    double turn_epsilon = epsilon;//2.0;
    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double turn_maximum_vel_adj = 5.0;
    //States the window in which the integral will be activated
    const double turn_integral_limit = 50.0;

    //Converts the Qangle to degrees
    double distance_in_degrees = angle.convert(degree);
    //Draws starting position from the gyro
    double start_pos_in_degrees = get_proper_gyro();
    //Calculates current position based on start position
    double current_pos_in_degrees = get_proper_gyro() - start_pos_in_degrees;
    //Sets the initial last pos to zero
    double last_current_pos_in_degrees = current_pos_in_degrees;
    //Sets the integral to zero so that additions can be made later
    double turn_integral = 0.0;


    double second_last_turn_error = 9999.9;

    double third_last_turn_error = 9999.9;

    double fourth_last_turn_error = 9999.9;

    //Defines the initial drive error
    double turn_error = distance_in_degrees - current_pos_in_degrees;

    //Sets last error to zero before turning starts
    double turn_last_error = turn_error;

    double last_three_turn_derivatives = 9999.9;



    //ACCELERATION FOR LATER
    //Sets the first speed to zero
    //double last_speed = 0.0;

    //Keep turning while the robot hasn't reached its target distance
    while ((fabs(last_three_turn_derivatives) > turn_epsilon) || (fabs(turn_error) > turn_epsilon*2.0))
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        // This code accounts for the fact that when the robot turns more than 360 degrees in one direction the gyroscope will still read proper values
        if(((current_pos_in_degrees + 180) < last_current_pos_in_degrees) && (distance_in_degrees > last_current_pos_in_degrees))
        {
          distance_in_degrees = distance_in_degrees - 360;
        }

        else if(((current_pos_in_degrees -180)> last_current_pos_in_degrees) && (distance_in_degrees < last_current_pos_in_degrees))
        {
          distance_in_degrees = distance_in_degrees + 360;
        }

        last_current_pos_in_degrees = current_pos_in_degrees;


        //Calculate distance left to turn
        turn_error = distance_in_degrees - current_pos_in_degrees;

        printf("current: %f  error: %f ", current_pos_in_degrees, turn_error);

        //Calculates the derivative
        double turn_derivative = turn_last_error - turn_error;

        fourth_last_turn_error = third_last_turn_error;

        third_last_turn_error = second_last_turn_error;

        second_last_turn_error = turn_last_error;

        //Sets a new last error
        turn_last_error = turn_error;

        double third_last_turn_derivative = fourth_last_turn_error - third_last_turn_error;

        double second_last_turn_derivative = third_last_turn_error - second_last_turn_error;

        double last_turn_derivative = second_last_turn_error - turn_last_error;

        last_three_turn_derivatives = last_turn_derivative + second_last_turn_derivative + third_last_turn_derivative;


        //Determines when the integral will start being used
        if(turn_ki != 0)
        {
            if(fabs(turn_error) < turn_integral_limit)
            {
                turn_integral = turn_integral + turn_error;
            }
            else
            {
                turn_integral = 0;
            }
        }
        else
        if((turn_integral > 0) && (turn_error < 0))
        {
          turn_integral = turn_integral * -1;
        }
        else if((turn_integral < 0) && (turn_error > 0))
        {
          turn_integral = turn_integral * -1;
        }

        //Calculate speed to be turned at using kp,ki,kd
        double speed = turn_error * turn_kp + turn_integral * turn_ki + turn_derivative * turn_kd;
        printf("stop: %f  Speed: %f  (p,i,d): (%f,%f,%f) ",last_three_turn_derivatives,speed,turn_error*turn_kp,turn_integral*turn_ki,turn_derivative*turn_kd);

        //Removes impossible speeds by setting the speed down to a possible one
        if(speed > max_speed)
        {
            speed = max_speed;
        }

        if(speed < max_speed * -1)
        {
            speed = max_speed * -1;
        }

        if ((speed > 0) && (speed < min_speed))
        {
          speed = min_speed;
        }
        if ((speed < 0) && (speed > -min_speed))
        {
          speed = -min_speed;
        }

        printf("adj speed: %f\n",speed);

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        //chassis.arcade(0.0, speed);
        chassis.arcade(0.0, speed);
        pros::delay(33);

        //Calculates current position based on start position after small movement
        current_pos_in_degrees =  get_proper_gyro() - start_pos_in_degrees;
    }

    //Stops the motors
    chassis.stop();
}


void gyro_turn_to(okapi::ChassisController& chassis, QAngle angle, double max_speed, double min_speed)
{
    double new_turn = angle.convert(degree) - get_proper_gyro();
    QAngle new_angle = new_turn * degree;
    gyro_turn(chassis, new_angle, max_speed, 17.5);
}
