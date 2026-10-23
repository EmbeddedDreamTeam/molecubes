#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "servo_tests.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "init_wifi.h"
#include "protocol_manager.h"
#include "buffer_header.h"
#include <task_manager.h>


void init_cube() {
    uint8_t mac_appo[6];
    esp_read_mac(mac_appo, ESP_MAC_WIFI_STA);
    ESP_LOGI("CUBE_INIT", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac_appo[0], mac_appo[1], mac_appo[2], mac_appo[3], mac_appo[4], mac_appo[5]);
    memcpy(mac, mac_appo, 6); //copying the mac address byte to byte to the molecube_data struct
}
//inizializza la logica del wifi, uart e del buffer dei comandi
//essenziale perchè il bridge wifi-uart carica i comandi in una coda che fa da buffer
// in modo che poi la task che invia i comandi al servo li esegua uno alla volta
// se in init_cmd_buffer() non si riesce a creare la coda viene lanciata un eccezione che blocca l'esecuzione
esp_err_t init_cmd_logic(){
    init_wifi();
    init_uart_comms();
    esp_err_t ris = init_cmd_buffer();
    return ris;
}

esp_err_t test_protocol_parser() {
    ProtocolManager::handle_incoming("M505");
    esp_err_t result= ESP_OK;
    if (ProtocolManager::handle_incoming("M24") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando M24.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("G6 N0 P1,5708 S1.0 A2.0 J3.0 N1 P0,785398 J1.5 N2 P0,785398 J1.5 N3 P0,785398 J1.5") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando G6 multiplo.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("G4 5000") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando G4.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("G6 N0 P0 S1.0 A2.0 J3.0") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando G6 singolo.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("M222 2.5") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando M222.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("M204 3.5") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando M204.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("M205 4.5") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando M205.");
        result = ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (ProtocolManager::handle_incoming("M505") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito a interpretare il comando M505.");
        result = ESP_FAIL;
    }
    return result;
}

esp_err_t test_commands_execution(){
    ProtocolManager::handle_incoming("M505");
    esp_err_t result=ESP_OK;
    //test comando G6
    ProtocolManager::handle_incoming("G6 N0 P1,5708");
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(4000));
    if (abs(servo_data.current_pos.load()-90.0f*(M_PI/180.0f))>0.1f || ack_to_receive.load() != 0){
        ESP_LOGE("TEST", "Fallito a eseguire il comando G6 N0 P1,5708");
        result=ESP_FAIL;
    }
    //test comando G6 per un servo diverso
    ProtocolManager::handle_incoming("G6 N1 P1,5708");
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(4000));
    if (ack_to_receive.load() != 0){
        ESP_LOGE("TEST", "Fallito a ricevere l'ack del comando G6 N1 P1,5708. ack_to_receive: %d", ack_to_receive.load());
        result=ESP_FAIL;
    }
    //test comando G4
    ProtocolManager::handle_incoming("G4 5000");
    ProtocolManager::handle_incoming("G6 N0 P0.0"); 
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (abs(servo_data.current_pos.load()-90.0f*(M_PI/180.0f))>0.1f){
        ESP_LOGE("TEST", "Fallito a eseguire il comando G4 5000");
        result=ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(8000));
    //test comando M505
    ProtocolManager::handle_incoming("M222 1.5");
    ProtocolManager::handle_incoming("M505");
    Msg* msg = nullptr;
    if (xQueueReceive(h_queue_cmd_buffer, &msg, 0) == pdTRUE|| ack_to_receive.load() != 0) {
        ESP_LOGE("TEST", "Ricevuto un comando dalla coda dopo M505, il buffer non è stato svuotato.");
        result=ESP_FAIL;
    }
    //test comando M24
    ProtocolManager::handle_incoming("G4 3000");
    ProtocolManager::handle_incoming("M24");
    //status è esposto con la compilazione condizionale
    if (status.load() != true|| ack_to_receive.load() != 0){
        ESP_LOGE("TEST", "Fallito a eseguire il comando M24, status: %d", status.load());
        result=ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(4000));
    //test comando M25
    ProtocolManager::handle_incoming("G4 1000");
    ProtocolManager::handle_incoming("G4 1000");
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(50));
    ProtocolManager::handle_incoming("M25");
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (stopped.load() != true|| ack_to_receive.load() != 0){
        ESP_LOGE("TEST", "Fallito a eseguire il comando M25, stopped: %d", stopped.load());
        result=ESP_FAIL;
    }
    return result;

}

esp_err_t test_commands_sequence(){
    ProtocolManager::handle_incoming("M505");
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("G6 N0 P0.0 N1 P0.0 N2 P0.0 N3 P0.0");
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("G6 N0 P-2,42601 N1 P-2,42601 N2 P-2,42601 N3 P-2,42601");
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("G6 N0 P-2,42601 N1 P-1,0472 N2 P0 N3 P0");
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("G6 N0 P-2,42601 N1 P-1,0472 N2 P-1,0472 N3 P-2,42601");
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(30000));
    if (ack_to_receive.load() != 0){
        ESP_LOGE("TEST", "Fallito a eseguire la sequenza di comandi, ack_to_receive: %d", ack_to_receive.load());
        return ESP_FAIL;
    }
    return ESP_OK;

}

esp_err_t test_buffer_overload(){
    ProtocolManager::handle_incoming("M505");
    for (int i = 0; i < 198; i++) {
        ProtocolManager::handle_incoming("G6 N0 P0 S1.0 A2.0 J3.0");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    Payload p{};
    Msg* msg = create_msg(SELF_ID.load(), SELF_ID.load(), type_servo, p);
    if (xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) {
        ESP_LOGE("TEST", "Fallito a eseguire il test di buffer overload, la coda non è piena dopo 198 comandi.");
        ProtocolManager::handle_incoming("M505");
        return ESP_FAIL;
    }
    ProtocolManager::handle_incoming("M505");
    return ESP_OK;
}

esp_err_t test_wrong_commands(){
    ProtocolManager::handle_incoming("M505");
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G6 senza parametri.");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6 N0") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G6 senza parametro P.");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6 N0 P0 J3.0 S1.0") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G6 con jerk prima di speed");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6 N0 P0 N1") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G6 con 2 servo senza il secondo parametro P");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G4") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G4 senza parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M222") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M222 senza parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M204") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M204 senza parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M205") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M205 senza parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6 N0 P0 K23") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G6 con un parametro sconosciuto K");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G4 5000 6000") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando G4 con due parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M222 1.5 2.5") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M222 con due parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M204 3.5 4.5") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M204 con due parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M205 4.5 5.5") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M205 con due parametri");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("M505");
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M24 K12") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M24 con un parametro sconosciuto K");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M25 K12") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M25 con un parametro sconosciuto K");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M505 K12") == ESP_OK) {
        ESP_LOGE("TEST", "Fallito: accettato il comando M505 con un parametro sconosciuto K");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("G6 N0 P0 ;ciao") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito: non accettato il comando G6 con un commento alla fine");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming(";ciao") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito: non accettata la riga di commento");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M222 2.0 ;ciao") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito: non accettato il comando M222 con un commento alla fine");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    ProtocolManager::handle_incoming("M505");
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ProtocolManager::handle_incoming("M25 ;ciao") != ESP_OK) {
        ESP_LOGE("TEST", "Fallito: non accettato il comando M25 con un commento alla fine");
        return ESP_FAIL;
    }
    return ESP_OK;

}


extern "C" void app_main() {
    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000, // Set the timeout to 10 seconds
        .idle_core_mask = (1 << 0), // Monitor all cores
        .trigger_panic = false // non panica in timeout
    };
    esp_task_wdt_init(&wdt_config);
    ESP_LOGI("TEST", "Starting servo tests...");
    init_cube();
    if (init_cmd_logic() != ESP_OK) {
        ESP_LOGE("TEST", "Failed to initialize command logic. Halting execution.");
        return; // Exit if initialization fails
    }
    servo_init();
    esp_err_t result=ESP_OK;
    result=test_protocol_parser()!=ESP_OK ? ESP_FAIL : result;
    vTaskDelay(pdMS_TO_TICKS(1000)); // aspetta un secondo per iniziare il test successivo
    result=test_commands_execution()!=ESP_OK ? ESP_FAIL : result;
    vTaskDelay(pdMS_TO_TICKS(1000)); // aspetta un secondo per iniziare il test successivo
    result=test_commands_sequence()!=ESP_OK ? ESP_FAIL : result;
    vTaskDelay(pdMS_TO_TICKS(1000)); // aspetta un secondo per iniziare il test successivo
    result=test_buffer_overload()!=ESP_OK ? ESP_FAIL : result;
    vTaskDelay(pdMS_TO_TICKS(1000)); // aspetta un secondo per iniziare il test successivo
    result=test_wrong_commands()!=ESP_OK ? ESP_FAIL : result;
    if (result==ESP_OK){
        ESP_LOGI("TEST", "Tutti i test completati con successo.");
    }else{
        ESP_LOGE("TEST", "Alcuni test sono falliti");
    }
    terminate_every_task();
}