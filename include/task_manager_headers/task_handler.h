#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace std;

//codice wifi
extern TaskHandle_t tcp_server_task_handle;

//codice uart
extern TaskHandle_t task_receive_uart_master_handle;
extern TaskHandle_t task_receive_uart_slave_handle;
extern TaskHandle_t task_send_uart_master_handle;
extern TaskHandle_t task_send_uart_slave_handle;
extern TaskHandle_t task_ping_slave_handle;
extern TaskHandle_t task_ping_master_handle;
extern TaskHandle_t task_loop_print_ids_array_handle;

//servo
extern TaskHandle_t move_servo_speed_task_handle;

//buffer
extern TaskHandle_t buffer_task_handle;