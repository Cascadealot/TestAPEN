/**
 * @file network_manager.c
 * @brief WiFi, OTA, and Debug Console Implementation
 */

#include "network_manager.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#include <string.h>
#include <stdarg.h>
#include <errno.h>

static const char *TAG = "NET";

// State
static bool g_wifi_connected = false;
static esp_netif_ip_info_t g_ip_info;
static network_command_callback_t g_cmd_callback = NULL;
static httpd_handle_t g_http_server = NULL;

// Debug server state
static int g_debug_socket = -1;
static int g_debug_client = -1;
static uint16_t g_debug_port = 2323;
static TaskHandle_t g_debug_task = NULL;

// Command buffer
static char g_cmd_buffer[256];
static size_t g_cmd_len = 0;

/*============================================================================
 * WiFi Event Handler
 *============================================================================*/

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                g_wifi_connected = false;
                ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
                esp_wifi_connect();
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        g_ip_info = event->ip_info;
        g_wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&g_ip_info.ip));
    }
}

/*============================================================================
 * OTA HTTP Handler
 *============================================================================*/

static esp_err_t ota_post_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "OTA update request, size=%d", req->content_len);

    esp_ota_handle_t ota_handle;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    if (update_partition == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition: %s", update_partition->label);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    char buf[1024];
    int received;
    int total = 0;

    while ((received = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        err = esp_ota_write(ota_handle, buf, received);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }
        total += received;

        // Progress every 64KB
        if ((total % 65536) == 0) {
            ESP_LOGI(TAG, "OTA progress: %d bytes", total);
        }
    }

    if (received < 0) {
        ESP_LOGE(TAG, "Error receiving data");
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
        return ESP_FAIL;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA success! Total: %d bytes. Rebooting...", total);
    httpd_resp_sendstr(req, "OTA Success! Rebooting...\n");

    // Reboot after response sent
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();

    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    const esp_app_desc_t *app_desc = esp_app_get_description();
    char response[256];
    snprintf(response, sizeof(response),
             "TestAPEN Status\n"
             "Version: %s\n"
             "IDF: %s\n"
             "IP: " IPSTR "\n",
             app_desc->version,
             app_desc->idf_ver,
             IP2STR(&g_ip_info.ip));
    httpd_resp_sendstr(req, response);
    return ESP_OK;
}

/*============================================================================
 * Coredump HTTP Handler - Download crash dump via OTA
 *============================================================================*/

static esp_err_t coredump_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Coredump download requested");

    // Find coredump partition
    const esp_partition_t *coredump_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);

    if (coredump_part == NULL) {
        ESP_LOGE(TAG, "Coredump partition not found");
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No coredump partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Coredump partition: offset=0x%lx, size=%lu",
             coredump_part->address, coredump_part->size);

    // Set response headers for binary download
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=coredump.bin");

    // Read and send partition contents in chunks
    char *buf = malloc(1024);
    if (buf == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    size_t offset = 0;
    size_t remaining = coredump_part->size;

    while (remaining > 0) {
        size_t to_read = (remaining > 1024) ? 1024 : remaining;

        esp_err_t err = esp_partition_read(coredump_part, offset, buf, to_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Partition read failed at offset %u: %s", offset, esp_err_to_name(err));
            free(buf);
            return ESP_FAIL;
        }

        if (httpd_resp_send_chunk(req, buf, to_read) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send chunk");
            free(buf);
            return ESP_FAIL;
        }

        offset += to_read;
        remaining -= to_read;
    }

    // End chunked response
    httpd_resp_send_chunk(req, NULL, 0);

    free(buf);
    ESP_LOGI(TAG, "Coredump sent: %lu bytes", coredump_part->size);
    return ESP_OK;
}

static void start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    if (httpd_start(&g_http_server, &config) == ESP_OK) {
        // OTA endpoint
        httpd_uri_t ota_uri = {
            .uri = "/update",
            .method = HTTP_POST,
            .handler = ota_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(g_http_server, &ota_uri);

        // Status endpoint
        httpd_uri_t status_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = status_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(g_http_server, &status_uri);

        // Coredump endpoint
        httpd_uri_t coredump_uri = {
            .uri = "/coredump",
            .method = HTTP_GET,
            .handler = coredump_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(g_http_server, &coredump_uri);

        ESP_LOGI(TAG, "HTTP server started on port 80");
        ESP_LOGI(TAG, "OTA: curl -X POST --data-binary @firmware.bin http://<ip>/update");
        ESP_LOGI(TAG, "Coredump: curl http://<ip>/coredump -o coredump.bin");
    }
}

/*============================================================================
 * Debug Console TCP Server
 *============================================================================*/

static void debug_server_task(void *pvParameters) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    g_debug_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (g_debug_socket < 0) {
        ESP_LOGE(TAG, "Failed to create debug socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(g_debug_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(g_debug_port);

    if (bind(g_debug_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind debug socket");
        close(g_debug_socket);
        g_debug_socket = -1;
        vTaskDelete(NULL);
        return;
    }

    listen(g_debug_socket, 1);
    ESP_LOGI(TAG, "Debug console listening on port %d", g_debug_port);

    while (1) {
        // Accept connection
        if (g_debug_client < 0) {
            g_debug_client = accept(g_debug_socket, (struct sockaddr *)&client_addr, &client_len);
            if (g_debug_client >= 0) {
                ESP_LOGI(TAG, "Debug client connected");
                g_cmd_len = 0;

                // Send welcome message
                const char *welcome = "TestAPEN Debug Console\r\nType 'help' for commands\r\n";
                send(g_debug_client, welcome, strlen(welcome), 0);
            }
        }

        // Read from client - use select() for efficient waiting
        if (g_debug_client >= 0) {
            fd_set readfds;
            struct timeval tv;
            FD_ZERO(&readfds);
            FD_SET(g_debug_client, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 50000;  // 50ms timeout

            int sel = select(g_debug_client + 1, &readfds, NULL, NULL, &tv);
            if (sel > 0 && FD_ISSET(g_debug_client, &readfds)) {
                // Data available - read in chunks
                char buf[64];
                int n = recv(g_debug_client, buf, sizeof(buf), 0);

                if (n > 0) {
                    for (int i = 0; i < n; i++) {
                        char c = buf[i];
                        if (c == '\n' || c == '\r') {
                            if (g_cmd_len > 0) {
                                g_cmd_buffer[g_cmd_len] = '\0';

                                // Process command
                                if (g_cmd_callback) {
                                    char response[1024];
                                    size_t resp_len = g_cmd_callback(g_cmd_buffer, response, sizeof(response));
                                    if (resp_len > 0) {
                                        send(g_debug_client, response, resp_len, 0);
                                    }
                                } else {
                                    // Default: echo command
                                    char resp[256];
                                    int len = snprintf(resp, sizeof(resp), "Unknown: %s\r\n", g_cmd_buffer);
                                    send(g_debug_client, resp, len, 0);
                                }
                                g_cmd_len = 0;
                            }
                        } else if (g_cmd_len < sizeof(g_cmd_buffer) - 1) {
                            g_cmd_buffer[g_cmd_len++] = c;
                        }
                    }
                } else if (n == 0) {
                    // Client disconnected
                    ESP_LOGI(TAG, "Debug client disconnected");
                    close(g_debug_client);
                    g_debug_client = -1;
                } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    // Socket error
                    ESP_LOGW(TAG, "Debug client error: %d", errno);
                    close(g_debug_client);
                    g_debug_client = -1;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/*============================================================================
 * Public API
 *============================================================================*/

esp_err_t network_manager_init(const char *ssid, const char *password,
                               const char *hostname, uint16_t debug_port) {
    esp_err_t err;

    // Initialize NVS (required for WiFi)
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Initialize network interface
    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Netif init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Create default event loop
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Event loop create failed: %s", esp_err_to_name(err));
        return err;
    }

    // Create default WiFi station
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    if (hostname && strlen(hostname) > 0) {
        esp_netif_set_hostname(netif, hostname);
    }

    // Initialize WiFi
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Register event handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, &instance_got_ip);

    // Configure WiFi
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) return err;

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK) return err;

    err = esp_wifi_start();
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", ssid);

    // Wait for connection (with timeout)
    int wait_count = 0;
    while (!g_wifi_connected && wait_count < 100) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
    }

    if (!g_wifi_connected) {
        ESP_LOGW(TAG, "WiFi connection timeout, will keep trying in background");
    }

    // Start HTTP server for OTA
    start_http_server();

    // Start debug console server
    g_debug_port = debug_port;
    xTaskCreate(debug_server_task, "debug_srv", 8192, NULL, 4, &g_debug_task);

    return ESP_OK;
}

void network_manager_set_command_callback(network_command_callback_t callback) {
    g_cmd_callback = callback;
}

bool network_manager_is_connected(void) {
    return g_wifi_connected;
}

void network_manager_get_ip(char *buf, size_t buf_size) {
    if (g_wifi_connected) {
        snprintf(buf, buf_size, IPSTR, IP2STR(&g_ip_info.ip));
    } else {
        snprintf(buf, buf_size, "Not connected");
    }
}

void network_manager_send(const char *msg) {
    if (g_debug_client >= 0) {
        send(g_debug_client, msg, strlen(msg), 0);
    }
}

void network_manager_printf(const char *fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (g_debug_client >= 0 && len > 0) {
        send(g_debug_client, buf, len, 0);
    }
}
