#include "wifi_manager.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

static const char* TAG = "WifiManager";

// Event handler: logga connessioni/disconnessioni dei client
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        auto* event = static_cast<wifi_event_ap_staconnected_t*>(event_data);
        ESP_LOGI(TAG, "Client connesso — MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        auto* event = static_cast<wifi_event_ap_stadisconnected_t*>(event_data);
        ESP_LOGI(TAG, "Client disconnesso — MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    }
}

void WifiManager::init_ap(const std::string& ssid, const std::string& password)
{
    // Initialize NVS (Non-Volatile Storage) required by the Wi-Fi driver for radio calibration data
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialize the TCP/IP stack (lwIP) and create the virtual interface for the Access Point
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create the system event loop to handle events like client connections/disconnections
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // Initialize Wi-Fi driver with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register an event handler to log when a computer (STA) connects or disconnects
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr, nullptr));

    // Configure Access Point parameters: SSID, Password, and maximum allowed connections
    wifi_config_t ap_config = {};
    strncpy(reinterpret_cast<char*>(ap_config.ap.ssid),
            ssid.c_str(), sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = static_cast<uint8_t>(ssid.size());
    ap_config.ap.max_connection = 4;  // massimo 4 client contemporanei

    if (password.empty()) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strncpy(reinterpret_cast<char*>(ap_config.ap.password),
                password.c_str(), sizeof(ap_config.ap.password));
        ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    // Set Wi-Fi mode to Access Point and start the radio
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access Point avviato — SSID: '%s'  IP: 192.168.4.1",
             ssid.c_str());
}


