#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "msg_structs.h"
#include <esp_log.h>
#include <stdexcept>
#include <atomic>
#include <utils_uart_comms.h>
#include "buffer_header.h"
#include "task_handler.h"

using namespace std;

QueueHandle_t h_queue_cmd_buffer;
std::atomic<int> ack_to_receive; //indice che indica quanti ack sono ancora da ricevere

//flag che indica lo stato della task. atomic per sicurezza e scalabilità
// true = la task processa i comandi, false = la task non processa i comandi
std::atomic<bool> status = false; 


std::atomic<bool> stopped=true; // Flag che indica lo start/stop manuale


void buffer_task(void *pvParameters) {
    uint8_t group_number=1;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); //necessaria per evitare che venga triggerato il WDT su burst di comandi
        if (status.load()==false){
            //se in stato di stop attende la ricezione del messaggio di start, anche in caso di timeout esegue il ciclo e ritorna qui
            if (ulTaskNotifyTake(pdTRUE,portMAX_DELAY) == 0x1) {
                ESP_LOGI("CMD_BUFFER", "task buffer ha ricevuto il comando di start, stopped=%d", stopped.load());
                // setta lo stato in esecuzione solo se non è in pausa manuale
                if (!stopped.load()) {
                    status.store(true);
                    ESP_LOGI("CMD_BUFFER", "Buffer task started processing commands.");
                }
            }
            else{
                //ricevuto comando di pausa manuale mentre la task era in attesa
            }
        }
        else{
            // se in stato di esecuzione prima di eseguire ogni comando controlla se è arrivato un comando di stop
            if (ulTaskNotifyTake(pdTRUE,0) == 0x2) {
                status.store(false);
                ESP_LOGI("CMD_BUFFER", "Buffer task stopped processing commands.");
            }
            if (status.load()==true){
                //se in stato di esecuzione esegue il prossimo comando
                Msg* msg;
                //processa i nuovi comandi solo se sono raggruppati o se non ci sono ack da ricevere
                ESP_LOGI("CMD_BUFFER", "ack_to_receive: %d, group_number: %d", ack_to_receive.load(), group_number);
                if (group_number>0||ack_to_receive.load()==0){
                    group_number=group_number>0? group_number-1:0;
                    // if (group_number==0){
                    //     //stoppa la task se il gruppo di comandi da processare è terminato
                    //     //la task verrà svegliata quando verranno ricevuti i messaggi di ack
                    //     status.store(false);
                    // }
                    if (xQueueReceive(h_queue_cmd_buffer, &msg, 0) == pdTRUE) {
                        // Process the received command
                        ESP_LOGD("CMD_BUFFER", "Processing command");
                        if (msg->type == type_group) {
                            ESP_LOGD("CMD_BUFFER", "Received group command with group_number=%d", msg->payload.payload_group.group_number);
                            group_number=msg->payload.payload_group.group_number;
                        }
                        else if (msg->type == type_servo) {
                            ESP_LOGD("CMD_BUFFER", "Sending servo command to target_id=%d", msg->target_id);
                            // gestione comando per il servo
                            //aumentando il numero di ack da ricevere,
                            ESP_LOGD("CMD_BUFFER", "ack_to_receive prima fetch_add: %d", ack_to_receive.load());
                            ack_to_receive.fetch_add(1);
                            ESP_LOGD("CMD_BUFFER", "ack_to_receive dopo fetch_add: %d", ack_to_receive.load());
                            if (msg->target_id == SELF_ID.load()) {
                                ESP_LOGD("CMD_BUFFER", "Command is for SELF_ID, sorting locally.");
                                // It's for the Root: send to the local servo queue
                                sort_new_msg(msg);
                            } else {
                                // It's for a Slave: route it through UART
                                send_msg_to_slave(msg);
                            }
                        }
                        else if (msg->type == type_g4) {
                            // blocca l'esecuzione per un certo numero
                            // vtaskdelay usa il numero di tick
                            ESP_LOGD("CMD_BUFFER", "Received G4 command, delaying for %d milliseconds", msg->payload.payload_g4.millis);
                            vTaskDelay(msg->payload.payload_g4.millis / portTICK_PERIOD_MS);
                            ESP_LOGD("CMD_BUFFER", "G4 delay completed.");
                            ESP_LOGD("CMD_BUFFER", "ack_to_receive: %d", ack_to_receive.load());
                            ESP_LOGD("CMD_BUFFER", "status: %d", status.load());
                        }
                    }
                    else{
                        // settando stato di stop, in questo modo la task resta in attesa
                        status.store(false);
                        stopped.store(true); // setta la pausa manuale a vero
                        send_sequence_ack(); //invia l'ack di fine sequenza
                    }
                }
                else{
                    // settando stato di stop, in questo modo la task resta in attesa degli ack
                    status.store(false);
                }
            }
        }
    }
}

esp_err_t init_cmd_buffer() {
    // la dimensione è 2 per sicurezza
    h_queue_cmd_buffer = xQueueCreate(200, sizeof(Msg*)); //! crea una coda con spazio per massimo 200 puntatori a Msg
    ack_to_receive.store(0); // inizializza il contatore degli ack da ricevere a 0
    BaseType_t result;
    result = xTaskCreate(buffer_task, "buffer_task", 4096, NULL, 5, &buffer_task_handle);
    if (h_queue_cmd_buffer == NULL) {
        // Handle error: Queue creation failed
        ESP_LOGE("CMD_BUFFER", "Failed to create command buffer queue");
        return ESP_FAIL;
    }
    if (result != pdPASS) {
        ESP_LOGE("CMD_BUFFER", "Failed to create buffer_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}