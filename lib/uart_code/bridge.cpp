#include "utils_uart_comms.h"
#include "esp_log.h"
#include "protocol_manager.h"
#include <cmath>
#include "buffer_header.h"
#include "task_handler.h"
using namespace std;



//*BRIDGE WIFI

// Helper to convert degrees to radians if your servo logic requires it
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
// questi valori verranno modificati dai comandi M222 M204 e M205
float default_speed = 1.0f;
float default_acc = 2.0f;
float default_jerk = 5.0f;

esp_err_t create_and_buffer_msg(module_id_t sender_id, module_id_t target_id, Payload& p){
    // se la posizione del servo non è valida e quindi target_id è -1 viene ignorato il messaggio
    if (target_id != -1){
        // non viene passato il puntatore a p, quindi viene eseguita una copia
        Msg* msg = create_msg(sender_id, target_id, type_servo, p);
        if (h_queue_cmd_buffer!=NULL){
            if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
                return ESP_FAIL; // questo errore verrà catturato in init_wifi.cpp
            }
        }
        else{
            ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
            free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
            return ESP_FAIL;
        }
    }
    return ESP_OK;

}



esp_err_t convert_servo_instructions(const Command& command){
    // qua ci sono solo comandi validi, quindi non serve fare controlli di validità
    int total_nodes = get_ids_array_len();
    module_id_t ids_arr[total_nodes];
    get_ids_array(ids_arr, total_nodes);


    Payload p{};
    p.payload_servo.radians = 0.0f;
    p.payload_servo.relative = false;
    p.payload_servo.speed = default_speed;
    p.payload_servo.acceleration = default_acc;
    p.payload_servo.jerk = default_jerk;
    int group_number=0;
    switch(command.gcode){
        case Gcode::G6:{
            // Handle G6 command specifics
            uint8_t target_id=0;
            bool relative=false; 
            if (command.args[0]==R){
                relative=true;
            }
            for (size_t i =0; i<command.args.size(); i++){
                if (command.args[i]==N){
                    //ignora il comando se la posizione del modulo non è valida
                    if (command.values[i]>=0 && command.values[i]<total_nodes){
                        group_number+=1;
                    }
                }
            }

            // creazione e buffering del comando group
            if (group_number>1){
                Payload group_payload{};
                group_payload.payload_group.group_number = group_number;
                Msg* msg = create_msg(SELF_ID.load(), SELF_ID.load(), type_group, group_payload);
                if (h_queue_cmd_buffer!=NULL){
                    if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                        ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                        free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
                        return ESP_FAIL; // questo errore verrà catturato in init_wifi.cpp
                    }
                }
                else{
                    ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
                    free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
                    return ESP_FAIL;
                }
            }

            for (size_t i = 0; i < command.args.size(); i++) { // salterà R se c'è 
                ESP_LOGD(
                    "SERVO_API",
                    "Processing arg %zu: %d with value %.3f",
                    i,
                    command.args[i],
                    command.values[i]
                );
                if (command.args[i] == Args::P) {
                    p.payload_servo.radians = command.values[i]; 
                } else if (command.args[i] == Args::S) {
                    // Handle S argument specifics
                    p.payload_servo.speed = command.values[i];
                } else if (command.args[i] == Args::A) {
                    // Handle A argument specifics
                    p.payload_servo.acceleration = command.values[i];
                } else if (command.args[i] == Args::J) {
                    // Handle J argument specifics
                    p.payload_servo.jerk = command.values[i];
                } else if (command.args[i] == Args::N) {
                    // setta il flag per il comando di posizione relativo
                    p.payload_servo.relative=relative;
                    // setta il flag per l'invio dell'ack
                    p.payload_servo.send_ack=true; 
                    if (i<2){
                        //dopo N è riportata la posizione del servo, che viene usata per identificare l'ID del servo
                        if (command.values[i]<0 || command.values[i]>=total_nodes){
                            target_id=-1; //posizione non valida
                        }
                        else{
                            //ottenendo l'id dalla posizione del servo
                            target_id = ids_arr[static_cast<int>(round(command.values[i]))];
                        } 
                    }
                    else{
                        ESP_LOGD(
                            "SERVO_API",
                            "total_nodes= %d, target_id=%d, angle=%.2f deg, radians=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f",
                            total_nodes,
                            target_id,
                            p.payload_servo.radians * (180.0f / M_PI), // Convert radians back to degrees for logging
                            p.payload_servo.radians,
                            p.payload_servo.speed,
                            p.payload_servo.acceleration,
                            p.payload_servo.jerk
                        );
                        //inserimento del comando nel buffer
                        if (target_id == SELF_ID.load()) {
                            
                            if (create_and_buffer_msg(SELF_ID.load(), SELF_ID.load(), p)!= ESP_OK){
                                return ESP_FAIL;
                            }

                        } else {
                            if (create_and_buffer_msg(SELF_ID.load(), target_id, p)!= ESP_OK){
                                return ESP_FAIL;
                            }
                        }
                        p = {}; // resettando i valori del payload
                        p.payload_servo.speed = default_speed;
                        p.payload_servo.acceleration = default_acc;
                        p.payload_servo.jerk = default_jerk;
                        p.payload_servo.radians = 0.0f;
                        if (command.values[i]<0 || command.values[i]>=total_nodes){
                            target_id=-1; //posizione non valida
                        }
                        else{
                            target_id = ids_arr[static_cast<int>(round(command.values[i]))];
                        } 
                    }
                }
            }
            
            // setta il flag per il comando di posizione relativo
            p.payload_servo.relative=relative;
            p.payload_servo.send_ack=true; // setta il flag per l'invio dell'ack
            if (target_id == SELF_ID.load()) {
                if (create_and_buffer_msg(SELF_ID.load(), SELF_ID.load(), p)!= ESP_OK){
                    return ESP_FAIL;
                }
            } else {
                if (create_and_buffer_msg(SELF_ID.load(), target_id, p)!= ESP_OK){
                    return ESP_FAIL;
                }
            }
            ESP_LOGD(
                "SERVO_API",
                "target_id=%d, angle=%.2f deg, radians=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f",
                target_id,
                p.payload_servo.radians * (180.0f / M_PI), // Convert radians back to degrees for logging
                p.payload_servo.radians,
                p.payload_servo.speed,
                p.payload_servo.acceleration,
                p.payload_servo.jerk
            );
            break;
        }
        case Gcode::M222:{
            // Handle M222 command
            default_speed = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default speed to %.3f", default_speed);
            break;
        }
        case Gcode::M204:{
            // Handle M204 command
            default_acc = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default acceleration to %.3f", default_acc);
            break;
        }
        case Gcode::M205:{
            // Handle M205 command
            default_jerk = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default jerk to %.3f", default_jerk);
            break;
        }
        case Gcode::G4:{
            // Handle G4 command
            ESP_LOGI("SERVO_API", "G4 command received");
            p.payload_g4.millis = static_cast<uint32_t>(command.values[0]);
            // It's for the Root: send to the local servo queue
            Msg* msg = create_msg(SELF_ID.load(), SELF_ID.load(), type_g4, p);
            if (h_queue_cmd_buffer!=NULL){
                if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                    ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                    free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
                    return ESP_FAIL; // questa eccezione verrà catturata in init_wifi.cpp
                }
            }
            else{
                ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
                free_msg(msg); // liberando il messaggio in caso la coda sia piena per evitare memory leak
                return ESP_FAIL;
            }
            break;
        }
        case Gcode::M24:{
            // Handle M24 command
            if (stopped.load()) {
                stopped.store(false); // setta la pausa manuale a falso
                if (xTaskNotify(buffer_task_handle, 0x1, eSetValueWithOverwrite) != pdPASS) {
                    ESP_LOGE("SERVO_API", "Failed to notify buffer task.");
                }
                else{
                    ESP_LOGI("SERVO_API", "Buffer task notified successfully.");
                }
            }
            break;
        }
        case Gcode::M25:{
            // Handle M25 command
            if (!stopped.load()) {
                stopped.store(true); // setta la pausa manuale a vero
                if (xTaskNotify(buffer_task_handle, 0x2, eSetValueWithOverwrite) != pdPASS) {
                    ESP_LOGE("SERVO_API", "Failed to notify buffer task.");
                }
                else{
                    ESP_LOGI("SERVO_API", "Buffer task notified successfully.");
                }
            }
            break;
        }
        case Gcode::M505:{
            // Handle M505 command
            xQueueReset(h_queue_cmd_buffer); // Reset the command buffer queue
            ack_to_receive.store(0); // Reset the ack counter
            stopped.store(true); 
            ESP_LOGI("SERVO_API", "Command buffer svuotato");
            break;
        }
        
    }
    return ESP_OK;
}

