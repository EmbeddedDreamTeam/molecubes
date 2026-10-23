#include "tcp_server.h"

#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "task_handler.h"

static const char* TAG = "TcpServer";

static constexpr int BUF_SIZE   = 1024;
static constexpr int TASK_STACK = 8192;

namespace {
    static TcpReceiveCallback s_on_receive;
    static TcpConnectCallback s_on_connect;
    static int  s_client_sock = -1;   // socket del client connesso (-1 = nessuno)
    static bool s_connected   = false;   // socket del client connesso (-1 = nessuno)

    // Task FreeRTOS: accept loop + ricezione dati
    static void tcp_server_task(void* arg)
    {
        
        auto port = static_cast<uint16_t>(reinterpret_cast<uintptr_t>(arg));

        // Create a streaming socket using the IPv4 TCP protocol
        int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Errore creazione socket: errno %d", errno);
            abort(); //fatal error, termina l'esecuzione e riavvia l'esp32
        }

        // Riutilizza la porta subito dopo una disconnessione
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in addr = {};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port        = htons(port);

        // Bind the socket to the static IP 192.168.4.1 and the specified port
        if (bind(listen_sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            ESP_LOGE(TAG, "Bind fallita: errno %d", errno);
            close(listen_sock);
            abort(); //fatal error, termina l'esecuzione e riavvia l'esp32
        }

        // Put the socket in listening mode to accept incoming connection requests
        listen(listen_sock, 1);
        ESP_LOGI(TAG, "Listening on 192.168.4.1:%d ...", port);

        while (true) {
            // Aspetta una connessione dal Mac
            sockaddr_in client_addr = {};
            socklen_t   client_len  = sizeof(client_addr);
            // Block execution until a client (computer) connects
            s_client_sock = accept(listen_sock,
                                   reinterpret_cast<sockaddr*>(&client_addr),
                                   &client_len);
            if (s_client_sock < 0) {
                ESP_LOGE(TAG, "Accept fallita: errno %d", errno);
                continue;
            }

            s_connected = true;
            ESP_LOGI(TAG, "Mac connesso — IP: %s",
                     inet_ntoa(client_addr.sin_addr));

            // Notify the application that the computer has successfully connected
            if (s_on_connect) s_on_connect();

            // Riceive data: accumulate string with '\n' termination 
            char        buf[BUF_SIZE];
            std::string line_buf;

            while (true) {
                // Read incoming data from the socket into the buffer
                int len = recv(s_client_sock, buf, sizeof(buf) - 1, 0);
                if (len <= 0) {
                    // Connession close or error
                    ESP_LOGI(TAG, "Mac disconected");
                    break;
                }

                // Process the buffer character by character to detect newlines ('\n')
                for (int i = 0; i < len; i++) {
                    char c = buf[i];
                    if (c == '\n') {
                        if (!line_buf.empty() && line_buf.back() == '\r') {
                            line_buf.pop_back();
                        }
                        if (!line_buf.empty() && s_on_receive) {
                            // When a full line is received, trigger the receive callback
                            s_on_receive(line_buf);
                        }
                        line_buf.clear();
                    } else {
                        line_buf += c;
                    }
                }
            }

            close(s_client_sock);
            s_client_sock = -1;
            s_connected   = false;

            ESP_LOGI(TAG, "In attesa di una nuova connessione...");
        }
    }
}

void TcpServer::start(uint16_t port, TcpReceiveCallback on_receive, TcpConnectCallback on_connect)
{
    s_on_receive = on_receive;
    s_on_connect = on_connect;

    // Passa la porta come argomento al task tramite cast (evita allocazione heap)
    xTaskCreate(tcp_server_task, "tcp_server", TASK_STACK,
                reinterpret_cast<void*>(static_cast<uintptr_t>(port)),
                5, &tcp_server_task_handle);
}

void TcpServer::send(const std::string& data)
{
    if (s_client_sock < 0) return;

    std::string msg = data + "\n";
    int sent = ::send(s_client_sock, msg.c_str(), msg.size(), 0);
    if (sent < 0) {
        ESP_LOGW(TAG, "Invio fallito: errno %d", errno);
    }
}

bool TcpServer::is_connected()
{
    return s_connected;
}