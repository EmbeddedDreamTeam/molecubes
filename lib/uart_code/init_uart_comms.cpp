#include "utils_uart_comms.h"
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_handler.h"

//* _______________________________________ MAIN e TEST
// void test_task(void* info){

//   int ct =0;
//   while(1){
//     Msg* prova = new Msg(); 
//     prova->header = HEADER_BYTE;
//     prova->footer = FOOTER_4_BYTES;
//     prova->sender_id = 0;
//     prova->target_id = 2;
//     prova->type = type_command_01;

//     char* s1 = prova->payload.payload_command_01.str1;
//     char* s2 = prova->payload.payload_command_01.str2;
//     sprintf(s1, "MGSN: %d", ct);
//     strcpy(s2, STR_PROVA);

//     printf("\nmetto in h_queue_send_to_slave: %p\n", (void*)prova);

//     send_msg_to_slave(prova);

//     vTaskDelay(pdMS_TO_TICKS(5000));
//     ct++;
//   }
// }


void task_loop_print_ids_array(void* info){
  while (1){
    print_ids_array();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}


esp_err_t init_uart_comms(){
  BaseType_t result;
  //todo for test  
  //!!!ID!!!
  string mac_str = "";
  for (int i = 0; i < 6; ++i) {
    char buf[3];
    sprintf(buf, "%02X", mac[i]);
    mac_str += buf;
  }
  if (mac_str == "0C4EA0653DD0"){
    SELF_ID.store(0);
  }
  else if (mac_str == "0C4EA0652684"){
    SELF_ID.store(1);
  }
  else if (mac_str == "0C4EA064B084"){
    SELF_ID.store(2);
  }
  else if (mac_str == "0C4EA0653E18"){
    SELF_ID.store(3);
  }
  else{
    ESP_LOGI("MAC","MAC Address not recognized: %s\n", mac_str.c_str());
    SELF_ID.store(-1); // Unknown ID
    return ESP_FAIL; // Return an error code to indicate failure
  }

  //*LOGS
  SHOW_UART_COMMS_LOGS = false;
  bool LOOP_PRINT_IDS_ARRAY = 0;

  //*TEST
  bool USE_DEFAULT_IDS = 0; //! == 1 => NON INIZIALIZZA la logica di handshake
  bool TEST_FUN = 0;
  int INIT_PAUSE = 1500;

  //*UART
  PRINT_RECEIVED_BYTES = 0;

  //*LED
  BLINK_ON_RECEIVE_MSG = 1;
  BLINK_ON_SEND_MSG = 1;

  int32_t LED_LOOP_DELAY = 5000;

  if(SELF_ID.load() == 0){ 
    MASTER_ID.store(-1);

    if(USE_DEFAULT_IDS) SLAVE_ID.store(1);
  }else if(SELF_ID.load() == 1){

    if(USE_DEFAULT_IDS){
      MASTER_ID.store(0);
      SLAVE_ID.store(2);
    }
  }else if(SELF_ID.load() == 2){
    if(USE_DEFAULT_IDS){
      MASTER_ID.store(2);
      SLAVE_ID.store(-1);
    }
  }

  //todo for test


  // esp_task_wdt_deinit();
  vTaskDelay(pdMS_TO_TICKS(INIT_PAUSE)); //!ATTENTO - RIMUOVERE O SETTARE A 1500
  if(SHOW_UART_COMMS_LOGS)
    printf("\nSTARTED init_uart_comms - STARTED init_uart_comms - STARTED init_uart_comms - STARTED init_uart_comms\n");

  //*LED SETUP
  init_led();
  set_loop_blink_delay(LED_LOOP_DELAY);
  //*LED DEFAULT BEHAVIOUR IS BLINKING IF NOTHING ELSE
  if(!BLINK_ON_RECEIVE_MSG){
    resume_loop_blink();
  }

  //*QUEUES
  //h_queue_ack = xQueueCreate(10, sizeof(Msg*));
  h_queue_debug = xQueueCreate(10, sizeof(Msg*));
  if (h_queue_debug == NULL) {
    ESP_LOGE("UART COMMS", "Failed to create h_queue_debug");
    return ESP_FAIL;
  }
  h_queue_handshake = xQueueCreate(10, sizeof(Msg*));
  if (h_queue_handshake == NULL) {
    ESP_LOGE("UART COMMS", "Failed to create h_queue_handshake");
    return ESP_FAIL;
  }
  h_queue_send_to_slave = xQueueCreate(10, sizeof(Msg*));
  if (h_queue_send_to_slave == NULL) {
    ESP_LOGE("UART COMMS", "Failed to create h_queue_send_to_slave");
    return ESP_FAIL;
  }
  h_queue_send_to_master = xQueueCreate(10, sizeof(Msg*));
  if (h_queue_send_to_master == NULL) {
    ESP_LOGE("UART COMMS", "Failed to create h_queue_send_to_master");
    return ESP_FAIL;
  }
  
  h_queue_servo = xQueueCreate(10, sizeof(Msg*));
  if (h_queue_servo == NULL) {
    ESP_LOGE("UART COMMS", "Failed to create h_queue_servo");
    return ESP_FAIL;
  }

  //*UART
  init_uart_mutexes();
  init_uart((uart_port_t)U_WITH_SLAVE, FROM_SLAVE_RX, TO_SLAVE_TX);
  //protezione per non inizializzare l'uart con il master se è la root
  if (SELF_ID.load() != ROOT_ID) {
    init_uart((uart_port_t)U_WITH_MASTER, FROM_MASTER_RX, TO_MASTER_TX);
    result = xTaskCreate(task_receive_uart, "task_receive_uart_master", 10000, (void*)U_WITH_MASTER, 2, &task_receive_uart_master_handle);
    if (result != pdPASS) {
      ESP_LOGE("UART COMMS", "Failed to create task_receive_uart_master");
      return ESP_FAIL;
    }
    result = xTaskCreate(task_send_uart, "task_send_uart_master", 10000, (void*)U_WITH_MASTER, 2, &task_send_uart_master_handle);
    if (result != pdPASS) {
      ESP_LOGE("UART COMMS", "Failed to create task_send_uart_master");
      return ESP_FAIL;
    }
    result = xTaskCreate(task_ping_master, "task_ping_master", 5000, nullptr, 2, &task_ping_master_handle);
    if (result != pdPASS) {
      ESP_LOGE("UART COMMS", "Failed to create task_ping_master");
      return ESP_FAIL;
    }
  }

  
  result = xTaskCreate(task_receive_uart, "task_receive_uart_slave", 10000, (void*)U_WITH_SLAVE, 2, &task_receive_uart_slave_handle);
  if (result != pdPASS) {
    ESP_LOGE("UART COMMS", "Failed to create task_receive_uart_slave");
    return ESP_FAIL;
  }

  result = xTaskCreate(task_send_uart, "task_send_uart_slave", 10000, (void*)U_WITH_SLAVE, 2, &task_send_uart_slave_handle);
  if (result != pdPASS) {
    ESP_LOGE("UART COMMS", "Failed to create task_send_uart_slave");
    return ESP_FAIL;
  }


  //*HANDSHAKE
  if (SELF_ID.load() == ROOT_ID) {
    module_id_t arr[3] = {0,1,2};
    init_report_handler(arr, 3, USE_DEFAULT_IDS);
  }
  if(!USE_DEFAULT_IDS){
    result = xTaskCreate(task_ping_slave, "task_ping_slave", 5000, nullptr, 2, &task_ping_slave_handle);
    if (result != pdPASS) {
      ESP_LOGE("UART COMMS", "Failed to create task_ping_slave");
      return ESP_FAIL;
    }
  }
  

  if(SELF_ID.load() == ROOT_ID && LOOP_PRINT_IDS_ARRAY){
    result = xTaskCreate(task_loop_print_ids_array, "task_loop_print_ids_array", 2000, nullptr, 5, &task_loop_print_ids_array_handle);
    if (result != pdPASS) {
      ESP_LOGE("UART COMMS", "Failed to create task_loop_print_ids_array");
      return ESP_FAIL;
    }
  }


  return ESP_OK;
}
