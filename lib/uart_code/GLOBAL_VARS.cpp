#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"

#include "msg_structs.h"
#include "utils_uart_comms.h"
#include "id_type.h"


uint8_t mac[6];
std::atomic<module_id_t> MASTER_ID = UNKNOWN_ID;
std::atomic<module_id_t> SELF_ID = UNKNOWN_ID;
std::atomic<module_id_t> SLAVE_ID = UNKNOWN_ID;

bool BLINK_ON_RECEIVE_MSG = false;
bool BLINK_ON_SEND_MSG = false;
bool PRINT_RECEIVED_BYTES = false;
bool SHOW_UART_COMMS_LOGS = false;

// TaskHandle_t h_task_blink_led_once;
//code x tutti i tipi di comandi diversi
QueueHandle_t h_queue_command_01;
QueueHandle_t h_queue_debug;
QueueHandle_t h_queue_handshake;
QueueHandle_t h_queue_servo;

//code x inviare messaggi
QueueHandle_t h_queue_send_to_slave;
QueueHandle_t h_queue_send_to_master;
