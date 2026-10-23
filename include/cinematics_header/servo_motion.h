#pragma once

#include "servo_types.h"

//  S-curve (jerk-limited) motion profile – state machine with 7 phases

//   ACCEL_JUP   jerk > 0 : acc  0  -> +a_max
//   ACCEL_CONST jerk = 0 : acc  = +a_max          (skipped if v_max < a²/j)
//   ACCEL_JDN   jerk < 0 : acc  +a_max -> 0
//   CRUISE      jerk = 0 : acc  = 0, vel = v_max  (skipped for short paths)
//   DECEL_JUP   jerk < 0 : acc  0  -> −a_max
//   DECEL_CONST jerk = 0 : acc  = −a_max          (skipped if vel_peak < a²/j)
//   DECEL_JDN   jerk > 0 : acc  −a_max -> 0

typedef enum {
    PH_ACCEL_JUP = 0,
    PH_ACCEL_CONST,
    PH_ACCEL_JDN,
    PH_CRUISE,
    PH_DECEL_JUP,
    PH_DECEL_CONST,
    PH_DECEL_JDN,
} MotionPhase;

#define min_speed 0.2f // minima velocità utilizzata per raggiungere il target in caso di undershoot

// motion profile helpers
//float decel_distance_sim(float v_init, float acc_init, float a_max, float j_max, float v_max);
float decel_distance(float v, float a_max, float j_max, float v_max);
float decel_distance_with_acc(float v, float a, float a_max, float j_max, float v_max);

