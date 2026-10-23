#include "utils_uart_comms.h"

//* _______________________________________LED
//GLOBAL VARS
bool LED_STATE = 0;
int BLINK_ONCE_DELAY = 350;
int BLINK_LOOP_DELAY = 1000;
TaskHandle_t h_task_blink_led_once;
TaskHandle_t h_task_blink_led_loop;



void toggle_led(bool s){ //1=ACCESO; 0=SPENTO
    gpio_set_level(LED_GPIO, !s);
    LED_STATE = s;
}


void task_blink_led_once(void *arg){
    
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        bool old_state = LED_STATE;
        if(old_state){
            toggle_led(0);
            vTaskDelay(pdMS_TO_TICKS(BLINK_ONCE_DELAY));
        }
        toggle_led(1);
        vTaskDelay(pdMS_TO_TICKS(BLINK_ONCE_DELAY));
        toggle_led(0);
        if(old_state){
            vTaskDelay(pdMS_TO_TICKS(BLINK_ONCE_DELAY));
            toggle_led(1);
        }

    }
}

void task_blink_led_loop(void *arg){
    vTaskSuspend(NULL);
    while(1){
        toggle_led(0);
        vTaskDelay(pdMS_TO_TICKS(BLINK_LOOP_DELAY));
        toggle_led(1);
        vTaskDelay(pdMS_TO_TICKS(BLINK_LOOP_DELAY));
        // printf("%d -- %d\n", SLAVE_ID, MASTER_ID);
    }
}

void wake_task_blink_led_once(uint32_t DELAY){
    if(DELAY > 0){
        BLINK_ONCE_DELAY = DELAY;
    }
    xTaskNotifyGive(h_task_blink_led_once);
}


void set_loop_blink_delay(uint32_t DELAY){
    BLINK_LOOP_DELAY = DELAY;
}

void resume_loop_blink(uint32_t DELAY){
    if(DELAY > 0){
        BLINK_LOOP_DELAY = DELAY;
    }
    vTaskResume(h_task_blink_led_loop);
}

void suspend_loop_blink(){
    vTaskSuspend(h_task_blink_led_loop);
}

void init_led(){
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    toggle_led(0);
    xTaskCreate(task_blink_led_once, "task_blink_led_once", 2048, nullptr, 24, &h_task_blink_led_once);
    xTaskCreate(task_blink_led_loop, "task_blink_led_loop", 2048, nullptr, 2, &h_task_blink_led_loop);
}