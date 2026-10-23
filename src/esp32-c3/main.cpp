#pragma GCC optimize ("Os")
#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "init_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "buffer_header.h"
#include "task_manager.h"


void init_cube() {
    uint8_t mac_appo[6];
    esp_read_mac(mac_appo, ESP_MAC_WIFI_STA);
    ESP_LOGI("CUBE_INIT", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac_appo[0], mac_appo[1], mac_appo[2], mac_appo[3], mac_appo[4], mac_appo[5]);
    memcpy(mac, mac_appo, 6); //copying the mac address byte to byte to the molecube_data struct
}


extern "C" void app_main() {
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reason_str;
    switch(reason){
        case ESP_RST_UNKNOWN: reason_str = "UNKNOWN"; break;
        case ESP_RST_POWERON: reason_str = "POWERON"; break;
        case ESP_RST_EXT: reason_str = "EXTERNAL_RESET"; break;
        case ESP_RST_SW: reason_str = "SOFTWARE_RESET"; break;
        case ESP_RST_PANIC: reason_str = "PANIC"; break;
        case ESP_RST_INT_WDT: reason_str = "INT_WDT"; break;
        case ESP_RST_TASK_WDT: reason_str = "TASK_WDT"; break;
        case ESP_RST_WDT: reason_str = "WDT"; break;
        case ESP_RST_DEEPSLEEP: reason_str = "DEEPSLEEP"; break;
        case ESP_RST_BROWNOUT: reason_str = "BROWNOUT"; break;
        case ESP_RST_SDIO: reason_str = "SDIO"; break;
        default: reason_str = "OTHER"; break;
    }
    ESP_LOGW("BOOT", "Reset reason: %d (%s)", reason, reason_str);
    
    //initializing uart comms, cube data (mac address) and servo controller
    init_cube();
    //il codice uart deve essere inizializzato prima del servo perchè li
    // avviene la creazione della coda da cui leggerà il servo e in cui
    // verrà inviato il comando di movimento iniziale del servo 
    ESP_ERROR_CHECK(init_uart_comms());
    ESP_ERROR_CHECK(servo_init());
    // in caso di errore viene chiamato abort() che termina tutte le task
}

