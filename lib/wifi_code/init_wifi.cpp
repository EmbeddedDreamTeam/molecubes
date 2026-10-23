#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wifi_manager.h"
#include "tcp_server.h"
#include "protocol_manager.h"
#include "utils_uart_comms.h"
#include "protocol_manager.h"

static const char* TAG = "Main";

// -----------------------------------------------------------------------
// Configurazione — modifica qui SSID, password, porta e numero di servos
// -----------------------------------------------------------------------
static constexpr char     AP_SSID[]     = "ESP32-C3-AP";
static constexpr char     AP_PASSWORD[] = "12345678";
static constexpr uint16_t TCP_PORT      = 3333;
// -----------------------------------------------------------------------

void init_wifi(){
    ESP_LOGI(TAG, "Avvio ESP32-C3 Wi-Fi TCP...");

    // Start the Access Point
    WifiManager::init_ap(AP_SSID, AP_PASSWORD);

    // Initialize the protocol and define what to do with the received motor data
    ProtocolManager::init(1, [](const Command& command) { //! Riceve i vettori di parametri e valori
        
        ESP_LOGI(TAG, "Comandi servo ricevuti:");
        for (int i = 0; i < (int)command.args.size(); i++) {
            // Uso %.1f perché i valori sono float. Modifica il numero dopo il punto per più o meno decimali.
            ESP_LOGI(TAG, " Comando: %s", gcodeToString[command.gcode].c_str());
        }

        // Forward parsed instructions to the UART bridge for physical motor control
        if (convert_servo_instructions(command) != ESP_OK) {
            reply("ERROR — unable to process command");
        }
    });

    // Start the TCP server and define the connection behavior
    TcpServer::start(TCP_PORT,
        [](const std::string& line) {               // on_receive
            // Route raw TCP strings to the protocol parser
            ProtocolManager::handle_incoming(line);
        },
        []() {
            // Automatically send the peripheral count to the computer upon connection
            reply("SERVOS " + std::to_string(s_num_servos));                                       // on_connect
        }
    );

    ESP_LOGI(TAG, "Pronto! Connettiti alla rete '%s' e usa:", AP_SSID);
    ESP_LOGI(TAG, "  nc 192.168.4.1 %d", TCP_PORT);

}