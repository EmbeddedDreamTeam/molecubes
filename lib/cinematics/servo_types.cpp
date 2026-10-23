#include "servo_types.h"
#include <math.h>

// single definition of servo_data (shared across translation units)
ServoData servo_data = {
    .duty_res = 0,                // will be set by servo_timer_init()
    .gpio = 5,
    .sgnl_min_duty = 500,
    .sgnl_max_duty = 2500,
    .min_pos = (float)(-30.5/36.0*M_PI) + trim,
    .max_pos = (float)( 30.5/36.0*M_PI) - trim,
    .current_pos = std::atomic<float>(0.0f),
    .current_speed = std::atomic<float>(0.0f),
    .current_acc = std::atomic<float>(0.0f),
    .max_speed = 5.2f,
    .max_acc = 10.0f,
    .max_jerk = 150.0f, 
    .moving = std::atomic<bool>(false),
};

float servo_deadzone = (270.0f/180.0f*M_PI)/(float)(servo_data.sgnl_max_duty-servo_data.sgnl_min_duty)*(float) servo_deadzone_ms;
