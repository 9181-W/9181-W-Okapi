#ifndef GYRO_H
#define GYRO_H

#include "okapi/api.hpp"
using namespace okapi;

void gyro_initialize();
void gyro_drive(okapi::ChassisController& chassis, QLength distance, double max_speed, bool drive_straight = true);
void gyro_turn(okapi::ChassisController& chassis, QAngle angle, double max_speed = 100, double min_speed = 17.5, double kp = 0.011, double ki = 0.0, double kd = 0.0, double epsilon = 2);
void gyro_turn_to(okapi::ChassisController& chassis, QAngle angle, double max_speed = 100, double min_speed = 17.5);
void async_gyro_drive(okapi::ChassisController& chassis, QLength distance, double max_speed);
bool drive_is_complete();
void wait_for_drive_complete();
void gyro_reset();

#endif
