#pragma once

#include <stdio.h>
#include <string.h>
#include <queue>
#include <vector>
#include <iostream>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include "freertos/semphr.h"
#include "protocol_manager.h"

//mine
#include "msg_structs.h"
#include "id_type.h"

using namespace std;



//* _______________________________________ CONSTS e STRUCTS
#define U_WITH_MASTER 0
#define U_WITH_SLAVE 1

#define FROM_MASTER_RX 10
#define TO_MASTER_TX 9
#define FROM_SLAVE_RX 2
#define TO_SLAVE_TX 3

#define U_BUF_SIZE 1024 
#define BAUD_RATE 115200

#define LED_GPIO GPIO_NUM_8 

//ids
#define ROOT_ID 0 
#define UNKNOWN_ID -1

//header e footer di ogni messaggio
#define HEADER_BYTE 0xAA
#define FOOTER_4_BYTES 0xCAFEBABE

#define STR_PROVA "messaggio_corretto"



typedef struct {
  uart_port_t select_uart; 
  QueueHandle_t select_queue;
} InfoUART;


//* GLOBAL_VARS.CPP
extern uint8_t mac[6];
extern std::atomic<module_id_t> MASTER_ID; 
extern std::atomic<module_id_t> SELF_ID;
extern std::atomic<module_id_t> SLAVE_ID;

extern bool BLINK_ON_RECEIVE_MSG; 
extern bool BLINK_ON_SEND_MSG;
extern bool PRINT_RECEIVED_BYTES;
extern bool SHOW_UART_COMMS_LOGS;

//handles:
// extern TaskHandle_t h_task_blink_led_once;

//code x tutti i tipi di comandi diversi
//extern QueueHandle_t h_queue_ack;
extern QueueHandle_t h_queue_debug;
extern QueueHandle_t h_queue_handshake;
extern QueueHandle_t h_queue_servo;



//code x inviare messaggi
extern QueueHandle_t h_queue_send_to_slave;
extern QueueHandle_t h_queue_send_to_master;

//semaforo per il report
extern SemaphoreHandle_t h_semaphore_report;


//* INIT_COMUNICATION.CPP
esp_err_t init_uart_comms();


//* LED.CPP
void toggle_led(bool s);
void init_led();
void wake_task_blink_led_once(uint32_t DELAY = -1);
void resume_loop_blink(uint32_t DELAY = -1);
void suspend_loop_blink();
void set_loop_blink_delay(uint32_t DELAY);



//* DEBUG_PRINT.CPP
void print_info_uart_struct(InfoUART* info);
const char* get_role_name(int role); //todo che tipo di int?
void print_msg_struct(Msg* msg);


//* UART.CPP
void sort_new_msg(Msg *msg);
void task_receive_uart(void *arg);
Msg* create_msg(module_id_t sender_id, module_id_t target_id, MsgType type, Payload payload);
// allocation helpers for Msg objects (centralize new/delete to detect double-frees)
Msg* allocate_msg();
void free_msg(Msg* msg);
void send_buffered_messages_to_master();
void send_msg_to_master(Msg* msg);
void send_buffered_messages_to_slave();
void send_msg_to_slave(Msg* msg);
void task_send_uart(void *arg);
void init_uart(uart_port_t uart_num, int rx_pin, int tx_pin);
void init_uart_mutexes();


//* HANDLE_HANDSHAKE.CPP
void task_ping_slave(void* info);
void task_ping_master(void* info);
void handle_handshakes(Msg* msg);
void send_report_to_root();


//*HANDLE_REPORT.CPP
void init_report_handler(module_id_t* default_ids, int default_ids_len, bool use_default_ids);
void receive_new_report(PayloadReport p);
int get_ids_array_len();
void get_ids_array(module_id_t* arr, int len);
void print_ids_array();


//* COMMANDS.CPP
void task_execute_command_01(void *arg);
void task_execute_debug(void *arg);
void task_execute_servo(void *arg);


//*BRIDGE_WIFI.CPP
esp_err_t convert_servo_instructions(const Command& command);
