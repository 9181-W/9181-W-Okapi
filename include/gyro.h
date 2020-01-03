#ifndef GYRO_H
#define GYRO_H

#include "okapi/api.hpp"
using namespace okapi;

void gyro_initialize();
void gyro_drive(okapi::ChassisController& chassis, QLength distance, double max_speed);
void gyro_turn(okapi::ChassisController& chassis, QAngle angle, double max_speed);

#endif
