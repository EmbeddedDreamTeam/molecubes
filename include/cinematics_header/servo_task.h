#pragma once

#include "servo_types.h"
#include "msg_structs.h"

// forward declarations
void move_servo_speed_task_state_machine(void * pvParameters);
ServoTaskParams sanitize_servo_command(Msg* msg);
esp_err_t servo_init();
void send_movement_ack();


