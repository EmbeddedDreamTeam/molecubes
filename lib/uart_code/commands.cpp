#include "utils_uart_comms.h"
#include "servo_controller.h"

//* _______________________________________ EXECUTE COMMANDS

//todo mockup only
void task_execute_command_01(void *arg){
  while(1){
    // Msg *msg = nullptr;
    // xQueueReceive(h_queue_command_01, &msg, portMAX_DELAY);
    // printf("START: execute_command_01\n");
    // vTaskDelay(pdMS_TO_TICKS(4000));
    // printf("END: execute_command_01\n");
    // delete msg; 
  }
}

void task_execute_debug(void *arg){
  while(1){
    Msg *msg = nullptr;
    xQueueReceive(h_queue_debug, &msg, portMAX_DELAY);
    printf("START: execute_debug\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    printf("END: execute_debug\n");
    delete msg; 
  }
}
