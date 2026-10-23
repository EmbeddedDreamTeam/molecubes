#include <stdio.h>
#include <string.h>

//definiti da me:
#include "msg_structs.h"
#include "utils_uart_comms.h"
#include "task_handler.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>

//* _______________________________________ GESTIONE DI HANDSHAKE



// proteggendo questa funzione con un semaforo si evita che 2 report possano essere inviati in ordine sbagliato
void send_report_to_root(){ 
  while(xSemaphoreTake(h_semaphore_report, portMAX_DELAY) != pdTRUE){
    // Wait until we can take the semaphore
  };
  ESP_LOGI("UART COMMS", "Sending report to root from module %d", SELF_ID.load());
  Payload p;
  p.payload_report.my_id = SELF_ID.load();
  p.payload_report.my_master_id = MASTER_ID.load();
  p.payload_report.my_slave_id = SLAVE_ID.load();

  // If I'm the root, handle the report locally
  if(SELF_ID.load() == ROOT_ID){
    receive_new_report(p.payload_report); //chiami direttamente il modulo
    xSemaphoreGive(h_semaphore_report);
    return;
  }

  // If I don't yet know my master, defer sending a report to avoid
  // producing stale reports that would later overwrite a fresher state
  // on the root (see issue with buffered reports).
  if(MASTER_ID.load() == UNKNOWN_ID){
    printf("[HANDSHAKE] MASTER unknown for SELF %d, deferring report\n", SELF_ID.load());
    xSemaphoreGive(h_semaphore_report);
    return;
  }

  Msg* m = create_msg(SELF_ID.load(), ROOT_ID, type_report, p);
  send_msg_to_master(m);
  xSemaphoreGive(h_semaphore_report);
}

const int PING_SLAVE_WAIT_FOR_ACK_MAX_DELAY = 1500; //! fix: do alle schedine 5s per rispondere ad un hanshake invece che 2s
const int PING_MASTER_WAIT_FOR_ACK_MAX_DELAY = 1500;

const int PING_SLAVE_SEND_NEW_HANDSHAKE_DELAY = 1500;
const int PING_MASTER_SEND_NEW_HANDSHAKE_DELAY = 1500;

std::atomic<module_id_t> last_MtS_ack_sender_id = -1;
std::atomic<bool> received_MtS_ack = false;

std::atomic<module_id_t> last_StM_ack_sender_id = -1;
std::atomic<bool> received_StM_ack = false;


void task_ping_slave(void* info){ // mando MtS a slave
  TickType_t before;
  TickType_t after;
  while(1){
    Payload p;
    p.payload_handshake.handshake_type = type_MtS;
    Msg* msg = create_msg(SELF_ID.load(), UNKNOWN_ID, type_handshake, p); 

    received_MtS_ack.store(false); 
    /*
    type_MtS ->
    type_StM <-
    TIMEOUT - NO RESPONSE
    Even if it doesnt have a response it knows not to delete the new slave
    */
    module_id_t SLAVE_ID_WHEN_I_SENT_THE_MESSAGE = SLAVE_ID.load(); 
    send_msg_to_slave(msg);
    before = xTaskGetTickCount();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PING_SLAVE_WAIT_FOR_ACK_MAX_DELAY)); //!NOTIFY
    after = xTaskGetTickCount();

    if(received_MtS_ack.load()){ // slave esiste
      if(last_MtS_ack_sender_id.load() != SLAVE_ID.load()){ // è diverso da slave ID
        printf(">>> SLAVE CHANGED FROM %d TO %d\n", SLAVE_ID.load(), last_MtS_ack_sender_id.load());

        SLAVE_ID.store(last_MtS_ack_sender_id.load()); 
        send_buffered_messages_to_slave();
        send_report_to_root();
      }
    } else if(SLAVE_ID_WHEN_I_SENT_THE_MESSAGE != UNKNOWN_ID) { // slave non esiste
      printf(">>> SLAVE %d DOESNT RESPOND, I ASSUME HE ISNT THERE (SLAVE_ID = -1)\n", SLAVE_ID.load());
      SLAVE_ID.store(UNKNOWN_ID); 
      send_report_to_root();
    }
    //attende il tempo rimanente dopo la risposta o il timeout
    TickType_t tick_to_wait = pdMS_TO_TICKS(PING_SLAVE_SEND_NEW_HANDSHAKE_DELAY);
    TickType_t remaining_ticks = (after - before) < tick_to_wait ? tick_to_wait - (after - before) : 0;
    vTaskDelay(remaining_ticks);
  }
}


void task_ping_master(void* info){ 
  if(SELF_ID.load() == ROOT_ID){ //it shouldn't be the case.
    task_ping_master_handle = NULL;
    vTaskDelete(nullptr);
  }
  TickType_t before;
  TickType_t after;
  while(1){
    Payload p;
    p.payload_handshake.handshake_type = type_StM;
    Msg* msg = create_msg(SELF_ID.load(), UNKNOWN_ID, type_handshake, p); 

    received_StM_ack.store(false); 
    module_id_t MASTER_ID_WHEN_I_SENT_THE_MESSAGE = MASTER_ID.load();
    send_msg_to_master(msg);
    before = xTaskGetTickCount();
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PING_MASTER_WAIT_FOR_ACK_MAX_DELAY)); //!NOTIFY
    after = xTaskGetTickCount();
    
    if(received_StM_ack.load()){ // master esiste
      if(last_StM_ack_sender_id.load() != MASTER_ID.load()){ // è diverso da master ID
        printf(">>> MASTER CHANGED FROM %d TO %d\n", MASTER_ID.load(), last_StM_ack_sender_id.load());

        MASTER_ID.store(last_StM_ack_sender_id.load()); 
        send_buffered_messages_to_master();
        send_report_to_root();
      }
    } else if(MASTER_ID_WHEN_I_SENT_THE_MESSAGE != UNKNOWN_ID) { // master non esiste
      printf(">>> MASTER %d DOESNT RESPOND, I ASSUME HE ISNT THERE (MASTER_ID = -1)\n", MASTER_ID.load());
      MASTER_ID.store(UNKNOWN_ID); 
      // todo NON MANDARE IL REPORT NON PUO GESTIRLO
    }

    //attende il tempo rimanente dopo la risposta o il timeout
    TickType_t tick_to_wait = pdMS_TO_TICKS(PING_SLAVE_SEND_NEW_HANDSHAKE_DELAY);
    TickType_t remaining_ticks = (after - before) < tick_to_wait ? tick_to_wait - (after - before) : 0;
    vTaskDelay(remaining_ticks);
  }
}


void handle_handshakes(Msg* msg){
  if(msg->payload.payload_handshake.handshake_type == type_MtS){ //* il master fa ciao rispondigli
    Payload p;
    p.payload_handshake.handshake_type = type_MtS_ack;
    Msg* nm = create_msg(SELF_ID.load(), UNKNOWN_ID, type_handshake, p);
    send_msg_to_master(nm);

    if(msg->sender_id != MASTER_ID.load()){
      if(SHOW_UART_COMMS_LOGS){
        printf(">>> MASTER CHANGED FROM %d TO %d\n", MASTER_ID.load(), msg->sender_id);}
      MASTER_ID.store(msg->sender_id); 
      send_buffered_messages_to_master(); //! fix: ho invertito questa righa e quella dopo
      send_report_to_root(); //so che non è -1 in quanto ho ricevuto un messaggio da qualcuno; 
    }

  } else if(msg->payload.payload_handshake.handshake_type == type_MtS_ack){
    last_MtS_ack_sender_id.store(msg->sender_id);
    received_MtS_ack.store(true); // FIX: consistency (true instead of 1)

    if(SHOW_UART_COMMS_LOGS)
      printf("DOVREI SVEGLIARMI\n");
    xTaskNotifyGive(task_ping_slave_handle); //ping_slave sends type_MtS
    if(SHOW_UART_COMMS_LOGS)
      printf("MI SONO SVEGLIATO\n");

  } else if(msg->payload.payload_handshake.handshake_type == type_StM){  //* lo slave fa ciao rispondigli
    Payload p;
    p.payload_handshake.handshake_type = type_StM_ack;
    Msg* nm = create_msg(SELF_ID.load(), UNKNOWN_ID, type_handshake, p);
    send_msg_to_slave(nm);

    if(msg->sender_id != SLAVE_ID.load()){
      if(SHOW_UART_COMMS_LOGS)
        printf(">>> SLAVE CHANGED FROM %d TO %d\n", SLAVE_ID.load(), msg->sender_id);
      SLAVE_ID.store(msg->sender_id); 
      send_buffered_messages_to_slave(); //! fix: ho invertito questa righa e quella dopo
      send_report_to_root();
    }

  } else if(msg->payload.payload_handshake.handshake_type == type_StM_ack){
    last_StM_ack_sender_id.store(msg->sender_id);
    received_StM_ack.store(true); 

    if(SHOW_UART_COMMS_LOGS)
      printf("DOVREI SVEGLIARMI\n");
    xTaskNotifyGive(task_ping_master_handle); //ping_master sends type_StM
    if(SHOW_UART_COMMS_LOGS)
      printf("MI SONO SVEGLIATO\n");
  }

  // msg was allocated with `new` in task_receive_uart/create_msg ->
  // must use `delete` to release it. `free` corrupts the C++ heap.
  free_msg(msg);
}

/*
X invia type_MtS / type_StM ciclicamente:
Y capisce chi è X, potrebbe essere cambiato, in caso aggiorna MY_MASTER e MY_SLAVE e manda un report a ROOT;
Y iniva type_MtS_ack / type_StM_ack a X (SOLO IN RISPOSTA AD UN type_MtS / type_StM);
X capisce chi è X, potrebbe essere cambiato, in caso aggiorna MY_MASTER e MY_SLAVE e manda un report a ROOT;
*/
