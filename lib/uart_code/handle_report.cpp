#include "msg_structs.h"
#include "utils_uart_comms.h"
#include "protocol_manager.h"

#include <iostream>
#include <string>
#include <array>
#include <cstdio>
#include <algorithm>
#include <task_handler.h>
#include <esp_log.h>
#include <unordered_map>

using namespace std;



vector<module_id_t> ids_array; //ordinati
int ids_array_len = 0;


unordered_map<module_id_t, PayloadReport> dict_map; //disordinati



const char* get_node_info(const PayloadReport& n){
    static char tmp[64];
    snprintf(tmp, sizeof(tmp), " {MASTER: %d, SELF: %d, SLAVE: %d}",
             n.my_master_id,
             n.my_id,
             n.my_slave_id);
    return tmp;
}

void print_node_info(PayloadReport& p){
    cout << get_node_info(p) << endl;
}


void print_ids_array(){
    cout << "PRINTING: IDS_ARRAY\n";
    for(int i = 0; i < ids_array_len; i++){
        cout << ids_array[i] << ' ';
    }
    cout << endl;
}


void compute_ids_array(){
    if (ids_array_len > 0) {
        ids_array.clear();
        ids_array_len = 0;
    }
    ids_array.push_back(ROOT_ID); // ROOT_ID == 0
    ids_array_len = 1; // reset the array

    // avoid loops
    unordered_map<module_id_t, bool> is_node_already_in_ids_array; 
    is_node_already_in_ids_array[ROOT_ID] = true;

    module_id_t curr_node_id = ROOT_ID;
    module_id_t next_node_id;
    PayloadReport current_node_info;
    unordered_map<module_id_t, PayloadReport>::iterator current_node_iter;
    while(true) {
        current_node_iter = dict_map.find(curr_node_id);
        if (current_node_iter == dict_map.end()) {
            break; // No report for the current node
        }
        current_node_info = current_node_iter->second;
        next_node_id = current_node_info.my_slave_id;
        ESP_LOGI("UART COMMS", "Current node: %d, Next node: %d", curr_node_id, next_node_id);
        if (next_node_id == UNKNOWN_ID|| is_node_already_in_ids_array[next_node_id]) {
            break;
        }
        ids_array.push_back(next_node_id);
        is_node_already_in_ids_array[next_node_id] = true;
        ids_array_len += 1;
        curr_node_id = next_node_id;
    }
}

//rimuove tutto il sottoalbero a partire da node_id, compreso node_id stesso
void remove_subtree_recursive(module_id_t node_id) {
    unordered_map<module_id_t, PayloadReport>::iterator node_iter = dict_map.find(node_id);
    if (node_iter == dict_map.end()) {
        return;
    }
    PayloadReport node_info=node_iter->second;
    remove_subtree_recursive(node_info.my_slave_id); //rimuovi il sottoalbero del figlio
    
    // rimuovi j
    dict_map.erase(node_id);
}


void receive_new_report(PayloadReport p){
    ESP_LOGI("UART COMMS", "Received new report from module %d: MASTER=%d, SLAVE=%d", p.my_id, p.my_master_id, p.my_slave_id);
    //se il valore è già presente viene aggiornato, altrimenti viene creato
    unordered_map<module_id_t, PayloadReport>::iterator slave_id_before_iter = dict_map.find(p.my_id);
    module_id_t slave_id_before;
    if (slave_id_before_iter == dict_map.end()) {
        slave_id_before = UNKNOWN_ID;
    } else{
        slave_id_before = slave_id_before_iter->second.my_slave_id;
    }
    dict_map[p.my_id] = p;
    ESP_LOGI("UART COMMS", "Updated dict for module %d: MASTER=%d, SLAVE=%d", p.my_id, dict_map.find(p.my_id)->second.my_master_id, dict_map.find(p.my_id)->second.my_slave_id);
    int len_before=get_ids_array_len();

    
    if (p.my_slave_id == UNKNOWN_ID&& slave_id_before != UNKNOWN_ID) {
        remove_subtree_recursive(slave_id_before);
    }

    compute_ids_array();
    if(SHOW_UART_COMMS_LOGS){
        cout << "RECEIVED NEW REPORT:" << endl;
        print_node_info(p);
        print_ids_array();
        cout << "___RECEIVED NEW REPORT:" << endl;
    }
    int len_after=get_ids_array_len();
    //manda un aggiornamento solo se è necessario
    if(len_before != len_after){
        ProtocolManager::set_num_servos((uint8_t)len_after);
    }
}


int get_ids_array_len(){
    return ids_array_len;
}

void get_ids_array(module_id_t* arr, int len){ //copia ids_array in arr
    for(int i=0; i<min(len, (int) ids_array.size()); i++){
        arr[i] = ids_array[i];
    }
}


void init_report_handler(module_id_t* default_ids, int default_ids_len, bool use_default_ids){

    if(use_default_ids){
        for(int i=0; i<default_ids_len; i++){
            ids_array.push_back(default_ids[i]);
        }
        ids_array_len=default_ids_len;

    }else{
        //*INIT IDS_ARRAY[] (ROOT IS ALONE)
        ids_array.push_back(ROOT_ID); //ROOT_ID == 0
        ids_array_len = 1; 

        //*INIT DICT[] e IS_DICT_EMPTY[] (ROOT IS ALONE)
        PayloadReport pr;
        pr.my_id = ROOT_ID;
        pr.my_master_id = UNKNOWN_ID;
        pr.my_slave_id = UNKNOWN_ID;
        dict_map[ROOT_ID] = pr;
    }
}