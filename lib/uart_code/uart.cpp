#include <stdio.h>
#include <string.h>

//definiti da me:
#include "msg_structs.h"
#include "utils_uart_comms.h"
#include "esp_log.h"
#include "buffer_header.h"
#include "task_handler.h"


//*GLOBALS
SemaphoreHandle_t master_buffer_mutex;
SemaphoreHandle_t slave_buffer_mutex;
SemaphoreHandle_t h_semaphore_report; //semaforo per la task che gestisce gli handshake


//* _______________________________________ ON START INIT UART


void init_uart_mutexes(){
    master_buffer_mutex = xSemaphoreCreateMutex();
    slave_buffer_mutex = xSemaphoreCreateMutex();
    h_semaphore_report = xSemaphoreCreateMutex();
}



void init_uart(uart_port_t uart_num, int rx_pin, int tx_pin) {
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122, 
        .source_clk = UART_SCLK_APB,
        .flags = 0,
    };
    
    uart_driver_install(uart_num, U_BUF_SIZE * 2, 0, 0, nullptr, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_pull_mode((gpio_num_t)rx_pin, GPIO_PULLUP_ONLY); 

    // if(SELF_ID == 1){ 
    //   printf("\nUART: %d, tx_pin: %d, rx_pin: %d\n", uart_num, tx_pin, rx_pin);
    // }
}



//* _______________________________________UART RECEIVE
// qua vengono gestiti i messaggi destinati a me, quelli che devono essere inoltrati vengono inviati direttamente senza passare da qui
void sort_new_msg(Msg *msg){ 
    if(msg->type == type_servo){
        ESP_LOGI("UART COMMS", "Sorting new message for target_id=%d, type=%d", msg->target_id, msg->type);
        xQueueSend(h_queue_servo, &msg, portMAX_DELAY);
    }else if (msg->type == type_debug){
        xQueueSend(h_queue_debug, &msg, portMAX_DELAY);
    }else if (msg->type == type_handshake){
        handle_handshakes(msg);
    }else if(msg->type == type_report){
        //chiamndo direttamente la funzione per gestire i report senza dover creare una task e una coda apposita
        receive_new_report(msg->payload.payload_report);
    }else if(msg->type == type_servo_ack){ 
        //decrementando il contatore di ack
        ESP_LOGI("UART COMMS", "Ack ricevuto da %d", msg->sender_id);
        ESP_LOGI("UART COMMS", "ack_to_receive prima: %d", ack_to_receive.load());
        if (ack_to_receive.load() > 0) {
            //è impossibile che ack_to_receive sia 0 quando viene ricevuto un ack,
            //perchè ack_to_receive viene incrementato prima dell'invio del messaggio.
            //questa protezione serve ad evitare casi di discrepanza dovuti a una possibile
            //ricezione di un comando M505 che resetta il contatore ack_to_receive
            //durante l'esecuzione di un movimento. Senza questa protezione
            //si potrebbe arrivare in uno stato impossibile in cui ack_to_receive è
            //negativo richiedendo l'invio di un altro comando M505 per sbloccare
            //la situazione.
            ack_to_receive.fetch_sub(1);
            ESP_LOGI("UART COMMS", "ack_to_receive dopo: %d", ack_to_receive.load());
        } else {
            ESP_LOGW("UART COMMS", "Received unexpected servo ack from %d, ack_to_receive is already 0", msg->sender_id);
        }
        if (ack_to_receive.load() == 0) {
            ESP_LOGI("UART COMMS", "buffer task svegliata");
            // sveglia la task del buffer dei comandi
            if (xTaskNotify(buffer_task_handle, 0x1, eSetValueWithOverwrite) != pdPASS) {
                ESP_LOGE("TEST", "Failed to notify buffer task.");
            }
        }
    }else{
        if(SHOW_UART_COMMS_LOGS)
            printf("ERRORE: [sort_new_msg] type: %i , non esiste\n", msg->type);
    }
}


void task_receive_uart(void *arg) { //todo fixare busy waiting, interrupt quando nel buffer ci sono n byte?
    uart_port_t selected_uart = (uart_port_t)(int32_t)arg;
    int flow_counter = 0; // Per distinguere i vari tentativi di ricezione

    while (1) {
        Msg *msg = new Msg();
        memset(msg, 0, sizeof(Msg));
        if(SHOW_UART_COMMS_LOGS)
            printf("[MSG_DBG] ALLOC msg=%p (zeroed)\n", (void*)msg); // Debug: log every allocation of a Msg

        bool message_ok = false;
        uint8_t *buf = (uint8_t*)msg;
        const size_t frame_len = sizeof(Msg);

        if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf("\n>>> [START FLOW #%d - UART %d] <<<\nRAW:", flow_counter++, (int)selected_uart);

        while (!message_ok) {
            size_t buffered_len = 0;
            if (uart_get_buffered_data_len(selected_uart, &buffered_len) == ESP_OK) {
                if (buffered_len > U_BUF_SIZE) {
                    if(SHOW_UART_COMMS_LOGS)
                        printf("\n[ALERT: Buffer overflow: %u byte]\n", (unsigned)buffered_len);
                }
            }

            //In loop finchè non trova l'header
            int r = 0;
            do {
                r = uart_read_bytes(selected_uart, buf, 1, portMAX_DELAY);
                if (r > 0) {
                    if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) {
                        printf(" %02X", buf[0]); // Stampa il byte ricevuto in esadecimale
                        fflush(stdout);
                    }
                }
            } while (buf[0] != HEADER_BYTE);
            
            if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf("|HEADER OK|"); 

            //Leggi il resto del messaggio
            bool do_start_over = false;
            size_t have = 1;
            while (have < frame_len) {
                int rr = uart_read_bytes(selected_uart, buf + have, frame_len - have, portMAX_DELAY);
                if (rr > 0) {
                    // Stampa i byte del corpo del messaggio
                    if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) {
                        for (int i = 0; i < rr; i++) {
                            printf(" %02X", buf[have + i]);
                        }
                        fflush(stdout);
                    }
                    have += (size_t)rr;
                } else {
                    if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf(" [THE THE MESSAGE IS INCOMPLETE]");
                    do_start_over = true;
                    break;
                }
            }

            
            if (do_start_over) {
                if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf("\n[I START OVER]\nRAW:");
                continue;
            }

            //Check footer
            if (msg->footer == FOOTER_4_BYTES) {
                if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf(" |OK|");
                message_ok = true;
            } else {
                if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf(" |BAD-FOOTER: %08X|, [I START OVER]", (unsigned int)msg->footer);
            }
        } // fine while(!message_ok)

        if (PRINT_RECEIVED_BYTES && SHOW_UART_COMMS_LOGS) printf("\n[FOOTER OK, MESSAGE OK]\n");


        //sort the message
        if(BLINK_ON_RECEIVE_MSG && msg->header == HEADER_BYTE && msg->footer == FOOTER_4_BYTES){
            wake_task_blink_led_once();
        }

        if(SHOW_UART_COMMS_LOGS)
            printf("\n================[RECEIVE UART]================\n");
        const char* role = get_role_name(selected_uart);
        if(msg->target_id == SELF_ID.load() || msg->target_id == -1){ //! in ogni caso se -1 lo prendo io
            if(SHOW_UART_COMMS_LOGS){
                printf("ID: %d | RICEVUTO DA: %s | DESTINAZIONE: ME\n", SELF_ID.load(), role);
                print_msg_struct(msg);
                printf("[MSG_DBG] sort_new_msg enqueue msg=%p type=%d sender=%d target=%d\n", (void*)msg, msg->type, msg->sender_id, msg->target_id);
            }
            sort_new_msg(msg);
        } else { 
            int uart_opposta = (int)!(bool)selected_uart;
            const char* tpr = get_role_name(uart_opposta);
            bool esiste = !(((int)uart_opposta == (int)UART_NUM_1 && SLAVE_ID.load() == -1) || ((int)uart_opposta == (int)UART_NUM_0 && MASTER_ID.load() == -1));
            
            if(SHOW_UART_COMMS_LOGS){
                printf("ID: %d | RICEVUTO DA: %s | FORWARD TO: %s (PRESENTE: %d)\n", SELF_ID.load(), role, tpr, esiste);
                print_msg_struct(msg);
                printf("[MSG_DBG] forwarding msg=%p to uart=%d (opposta=%d)\n", (void*)msg, (int)selected_uart, uart_opposta);
            }
            if(selected_uart == U_WITH_MASTER){
                send_msg_to_slave(msg);
            }else if(selected_uart == U_WITH_SLAVE){
                if (msg->type == type_report) {
                    //se ricevo un report dal mio slave (sono certo che sia lo slave perchè nel report c'è il mio id come master)
                    //e il suo id è diverso da quello che ho vuol dire che è arrivato prima il report dell'handshake.
                    //in questo caso allora aggiorno direttamente la topologia e invio un report aggiornato alla root
                    //prima di inviare il report del mio slave, in questo modo sono sicuro che alla root arriverà prima
                    //il report del mio modulo con lo slave aggiornato e poi il report del mio slave
                    // in questo modo è impossibile che venga distrutto il sottoalbero di moduli che ho sotto di me.
                    if (msg->payload.payload_report.my_master_id == SELF_ID.load() && SLAVE_ID.load() != msg->sender_id) {
                        ESP_LOGW("UART COMMS", "Received report from %d but SLAVE_ID is UNKNOWN.", msg->sender_id);
                        SLAVE_ID.store(msg->sender_id); 
                        send_buffered_messages_to_slave();
                        send_report_to_root();
                    }
                } 
                send_msg_to_master(msg);
            }
        }

        if(SHOW_UART_COMMS_LOGS){
            printf("================[__RECEIVE UART]================\n\n");
            fflush(stdout);
        }

    }
}


//* _______________________________________UART SEND
void task_send_uart(void *arg){
  uart_port_t selected_uart = (uart_port_t)(int32_t)arg;
  QueueHandle_t selected_queue = nullptr;
  if(selected_uart == U_WITH_MASTER){
      selected_queue = h_queue_send_to_master;
  }else if(selected_uart == U_WITH_SLAVE){
      selected_queue = h_queue_send_to_slave;
  }

  while (1) {
    Msg *msg = nullptr; 
    
    xQueueReceive(selected_queue, &msg, portMAX_DELAY);
    //todo check prima di inviare
    int bytes_sent = uart_write_bytes(selected_uart, (const void*)msg, sizeof(Msg));
    
    if(SHOW_UART_COMMS_LOGS){
        printf("\n================[SEND UART]================\n");
        const char* role = get_role_name(selected_uart);
        printf("SONO: %d, HO INVIATO INVIO A: %s, IL SEGUENTE MESSAGGIO:\n", SELF_ID.load(), role);
        print_msg_struct(msg);
        if (bytes_sent != sizeof(Msg)) {
            printf("ERRORE: inviati %d byte su %d\n", bytes_sent, sizeof(Msg));
        } else {
            printf("Tutti i byte inviati correttamente\n");
        }
    }

    if(BLINK_ON_SEND_MSG){
      wake_task_blink_led_once();
    }

    if(SHOW_UART_COMMS_LOGS){
        printf("================[__SEND UART]================\n\n");
        fflush(stdout);
    }

    free_msg(msg); 
  } 
}



Msg* create_msg(module_id_t sender_id, module_id_t target_id, MsgType type, Payload payload){
  Msg* msg = allocate_msg();
  // ensure header/footer/ids are set
  msg->header = HEADER_BYTE;
  msg->footer = FOOTER_4_BYTES;
  msg->sender_id = sender_id;
  msg->target_id = target_id;
  msg->type = type;
  msg->payload = payload; // il payload viene copiato perchè nella union le varie struct non contengono puntatori
  return msg;
}

// Centralized allocation/deallocation to avoid mixing new/free across the codebase
Msg* allocate_msg(){
    Msg* msg = new Msg();
    memset(msg, 0, sizeof(Msg));
    return msg;
}

void free_msg(Msg* msg){
    if(msg == nullptr){
        if(SHOW_UART_COMMS_LOGS)
            printf("WARNING: free_msg(): msg == nullptr\n");
    }else{
        if(SHOW_UART_COMMS_LOGS)
            printf("free_msg(): deleted %p\n", msg);
        delete msg;
        msg=nullptr; //per sicurezza e evitare dangling pointers
    }
}


// accumula tutti i messaggi che non puo inviare al suo master xche non lo conosce
queue<Msg*> master_pre_init_buffer; 
void send_buffered_messages_to_master(){ //todo fix busy waiting
    // Pop items from the buffered queue in a thread-safe way. Do NOT hold
    // the mutex while calling xQueueSend because xQueueSend can block and
    // would prevent other tasks from pushing into the buffer.
    xSemaphoreTake(master_buffer_mutex, portMAX_DELAY);
    while (1) {
        if (master_pre_init_buffer.empty() || MASTER_ID.load() == UNKNOWN_ID) {
            xSemaphoreGive(master_buffer_mutex);
            break;
        }
        Msg* m = master_pre_init_buffer.front();
        master_pre_init_buffer.pop();
        xQueueSend(h_queue_send_to_master, &m, portMAX_DELAY); //! fix: ho rimesso questa riga dentro al semaforo
    }
    xSemaphoreGive(master_buffer_mutex);
}


void send_msg_to_master(Msg* msg){
    xSemaphoreTake(master_buffer_mutex, portMAX_DELAY);
    //guardia per evitare che la base, nel caso in cui riceve un messaggio sbagliato
    //intasi il buffer del master
    if (SELF_ID.load() == ROOT_ID) {
        ESP_LOGW("UART COMMS", "send_msg_to_master called on ROOT_ID, message will not be sent.");
        free_msg(msg); // Avoid memory leak
        return;
    }
    if(MASTER_ID.load() == UNKNOWN_ID && msg->type != type_handshake){ //!type_handshake passa in ogni caso
        master_pre_init_buffer.push(msg);
    } else {
        xQueueSend(h_queue_send_to_master, &msg, portMAX_DELAY);
    }
    xSemaphoreGive(master_buffer_mutex);
}



// accumula tutti i messaggi che non puo inviare al suo slave xche non lo conosce
queue<Msg*> slave_pre_init_buffer; 
void send_buffered_messages_to_slave(){
    // Same pattern as master: protect std::queue with semaphore but do the
    // potentially-blocking xQueueSend outside the critical section.
    xSemaphoreTake(slave_buffer_mutex, portMAX_DELAY);
    while (1) {
        if (slave_pre_init_buffer.empty() || SLAVE_ID.load() == UNKNOWN_ID) {
            xSemaphoreGive(slave_buffer_mutex);
            break;
        }
        Msg* m = slave_pre_init_buffer.front();
        slave_pre_init_buffer.pop();
        xQueueSend(h_queue_send_to_slave, &m, portMAX_DELAY); //! fix: ho rimesso questa riga dentro al semaforo
    }
    xSemaphoreGive(slave_buffer_mutex);
}

 //todo aggiungere guardia per il report, perchè è vero che non viene generato
 //se un cubo ha un master sconosciuto, ma se un cubo che ha il master sconosciuto
 //riceve un report da un altro cubo, lo inoltra al suo master
 //sconosciuto e quindi possono essere inviate informazioni errate.
void send_msg_to_slave(Msg* msg){
    xSemaphoreTake(slave_buffer_mutex, portMAX_DELAY);
    if(SLAVE_ID.load() == UNKNOWN_ID && msg->type != type_handshake){ //!type_handshake passa in ogni caso
        slave_pre_init_buffer.push(msg);
    } else {
        xQueueSend(h_queue_send_to_slave, &msg, portMAX_DELAY);
    }
    xSemaphoreGive(slave_buffer_mutex);
}
