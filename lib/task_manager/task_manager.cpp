#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_manager.h"
#include "task_handler.h"
#include <esp_log.h>

using namespace std;

//codice wifi
TaskHandle_t tcp_server_task_handle= NULL;

//codice uart
TaskHandle_t task_receive_uart_master_handle= NULL;
TaskHandle_t task_receive_uart_slave_handle= NULL;
TaskHandle_t task_send_uart_master_handle= NULL;
TaskHandle_t task_send_uart_slave_handle= NULL;
TaskHandle_t task_ping_slave_handle= NULL;
TaskHandle_t task_ping_master_handle= NULL;
TaskHandle_t task_loop_print_ids_array_handle= NULL;

//servo
TaskHandle_t move_servo_speed_task_handle= NULL;

//buffer
TaskHandle_t buffer_task_handle=NULL;


void terminate_every_task(){
    //la memoria usasta dalle altre task è automaticamente liberata dalla API di FreeRTOS
    if (tcp_server_task_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating TCP server task...");
        vTaskDelete(tcp_server_task_handle);
        tcp_server_task_handle = NULL;
    }
    if (task_receive_uart_master_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART receive master task...");
        vTaskDelete(task_receive_uart_master_handle);
        task_receive_uart_master_handle = NULL;
    }
    if (task_receive_uart_slave_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART receive slave task...");
        vTaskDelete(task_receive_uart_slave_handle);
        task_receive_uart_slave_handle = NULL;
    }
    if (task_send_uart_master_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART send master task...");
        vTaskDelete(task_send_uart_master_handle);
        task_send_uart_master_handle = NULL;
    }
    if (task_send_uart_slave_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART send slave task...");
        vTaskDelete(task_send_uart_slave_handle);
        task_send_uart_slave_handle = NULL;
    }
    if (task_ping_slave_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART ping slave task...");
        vTaskDelete(task_ping_slave_handle);
        task_ping_slave_handle = NULL;
    }
    if (task_ping_master_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART ping master task...");
        vTaskDelete(task_ping_master_handle);
        task_ping_master_handle = NULL;
    }
    if (task_loop_print_ids_array_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating UART loop print ids array task...");
        vTaskDelete(task_loop_print_ids_array_handle);
        task_loop_print_ids_array_handle = NULL;
    }
    if (move_servo_speed_task_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating servo speed task...");
        vTaskDelete(move_servo_speed_task_handle);
        move_servo_speed_task_handle = NULL;
    }
    if (buffer_task_handle != NULL) {
        ESP_LOGI("TASK_MANAGER", "Terminating buffer task...");
        vTaskDelete(buffer_task_handle);
        buffer_task_handle = NULL;
    }

}