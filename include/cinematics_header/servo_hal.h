#pragma once

#include "servo_types.h"
#include "esp_err.h"

void servo_timer_init();
//converting degrees to radians
float rad_from_deg(int32_t degrees);
esp_err_t set_servo_pos(float rad);

