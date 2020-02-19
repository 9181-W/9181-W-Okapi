#ifndef UTILS_H
#define UTILS_H

#include "okapi/api.hpp"
using namespace okapi;

extern okapi::Motor a_mtr;
extern okapi::Motor t_mtr;
extern okapi::Motor li_mtr;
extern okapi::Motor ri_mtr;
extern okapi::Motor lf_mtr;
extern okapi::Motor lr_mtr;
extern okapi::Motor rf_mtr;
extern okapi::Motor rr_mtr;

extern okapi::AsyncPosIntegratedController tray;

extern okapi::AsyncVelIntegratedController intakes;

extern okapi::AsyncPosIntegratedController arm;

extern okapi::ChassisControllerIntegrated chassis;

void init_auto();

void tray_return();
void tray_return_fast();
void tray_up();
void three_cubes();
void six_cubes();
void nine_cubes();
void bottom();
void alliance_tower();
void short_tower();
void medium_high_tower();
void deploy();
void forty_five_deg_turn(QAngle angle = 45_deg);
void fifty_deg_to_eighty_deg_turn();
void ninety_deg_turn();
void one_hundred_eighty_deg_turn();
void intake_on(double speed = 180.0);
void intake_off();
void nine_cube_place();
void edited_nine_cube_place();
void edited_seven_cube_place();
void seven_cube_place();
void arm_pos_P(double arm_target_position, double arm_max_speed);
void tray_stack(double tray_target_position, double tray_max_speed);

#endif
