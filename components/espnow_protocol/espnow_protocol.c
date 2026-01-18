/**
 * @file espnow_protocol.c
 * @brief ESP-NOW Protocol Implementation for TestAPEN
 */

#include "espnow_protocol.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "string.h"
#include "sdkconfig.h"

static const char *TAG = "espnow";

/*============================================================================
 * Static Variables
 *============================================================================*/

static bool s_initialized = false;
static espnow_recv_cb_t s_recv_callback = NULL;
static uint8_t s_own_mac[6] = {0};

// Peer MAC addresses parsed from Kconfig
static uint8_t s_master_mac[6] = {0};
static uint8_t s_rudder_mac[6] = {0};
static uint8_t s_ui_mac[6] = {0};

// Statistics
static uint32_t s_tx_success_count = 0;
static uint32_t s_tx_fail_count = 0;
static uint32_t s_rx_count = 0;
static uint32_t s_rx_error_count = 0;

// Broadcast MAC for sending to all
static const uint8_t s_broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*============================================================================
 * Helper Functions
 *============================================================================*/

void int16_to_be(int16_t value, uint8_t *buf)
{
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
}

int16_t be_to_int16(const uint8_t *buf)
{
    return (int16_t)((buf[0] << 8) | buf[1]);
}

void int32_to_be(int32_t value, uint8_t *buf)
{
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}

int32_t be_to_int32(const uint8_t *buf)
{
    return (int32_t)((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]);
}

void espnow_mac_to_str(const uint8_t *mac, char *buf)
{
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Parse MAC address string "AA:BB:CC:DD:EE:FF" to bytes
 */
static bool parse_mac_string(const char *str, uint8_t *mac)
{
    if (!str || strlen(str) != 17) {
        return false;
    }

    unsigned int tmp[6];
    if (sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
               &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5]) != 6) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)tmp[i];
    }
    return true;
}

/**
 * @brief Check if MAC is all zeros
 */
static bool mac_is_zero(const uint8_t *mac)
{
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0) return false;
    }
    return true;
}

/**
 * @brief Check if MAC matches our own
 */
static bool mac_is_own(const uint8_t *mac)
{
    return memcmp(mac, s_own_mac, 6) == 0;
}

/*============================================================================
 * ESP-NOW Callbacks
 *============================================================================*/

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_tx_success_count++;
    } else {
        s_tx_fail_count++;
        ESP_LOGD(TAG, "Send failed to " MACSTR, MAC2STR(mac_addr));
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                            const uint8_t *data, int len)
{
    if (!recv_info || !data || len < 1) {
        s_rx_error_count++;
        return;
    }

    s_rx_count++;

    // Parse frame: first byte is msg_type, rest is payload
    uint8_t msg_type = data[0];
    const uint8_t *payload = data + 1;
    size_t payload_len = len - 1;

    // Cap payload length to our max
    if (payload_len > ESPNOW_MAX_PAYLOAD) {
        payload_len = ESPNOW_MAX_PAYLOAD;
    }

    // Call user callback if registered
    if (s_recv_callback) {
        s_recv_callback(msg_type, payload, payload_len, recv_info->src_addr);
    }
}

/*============================================================================
 * Peer Management
 *============================================================================*/

static esp_err_t add_peer_if_valid(const uint8_t *mac, const char *name)
{
    if (mac_is_zero(mac)) {
        ESP_LOGW(TAG, "Peer %s has zero MAC, skipping", name);
        return ESP_OK;
    }

    if (mac_is_own(mac)) {
        ESP_LOGD(TAG, "Peer %s is our own MAC, skipping", name);
        return ESP_OK;
    }

    // Check if already exists
    if (esp_now_is_peer_exist(mac)) {
        ESP_LOGD(TAG, "Peer %s already exists", name);
        return ESP_OK;
    }

    esp_now_peer_info_t peer = {
        .channel = 0,  // Use current channel
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, 6);

    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer %s: %s", name, esp_err_to_name(err));
        return err;
    }

    char mac_str[18];
    espnow_mac_to_str(mac, mac_str);
    ESP_LOGI(TAG, "Added peer %s: %s", name, mac_str);
    return ESP_OK;
}

/*============================================================================
 * Public API
 *============================================================================*/

esp_err_t espnow_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing ESP-NOW protocol");

    // Get own MAC address
    esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, s_own_mac);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get own MAC: %s", esp_err_to_name(err));
        return err;
    }

    char mac_str[18];
    espnow_mac_to_str(s_own_mac, mac_str);
    ESP_LOGI(TAG, "Own MAC: %s", mac_str);

    // Parse peer MACs from Kconfig
#ifdef CONFIG_ESPNOW_MASTER_MAC
    if (!parse_mac_string(CONFIG_ESPNOW_MASTER_MAC, s_master_mac)) {
        ESP_LOGW(TAG, "Invalid Master MAC in config: %s", CONFIG_ESPNOW_MASTER_MAC);
    }
#endif

#ifdef CONFIG_ESPNOW_RUDDER_MAC
    if (!parse_mac_string(CONFIG_ESPNOW_RUDDER_MAC, s_rudder_mac)) {
        ESP_LOGW(TAG, "Invalid Rudder MAC in config: %s", CONFIG_ESPNOW_RUDDER_MAC);
    }
#endif

#ifdef CONFIG_ESPNOW_UI_MAC
    if (!parse_mac_string(CONFIG_ESPNOW_UI_MAC, s_ui_mac)) {
        ESP_LOGW(TAG, "Invalid UI MAC in config: %s", CONFIG_ESPNOW_UI_MAC);
    }
#endif

    // Initialize ESP-NOW
    err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Register callbacks
    err = esp_now_register_send_cb(espnow_send_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register send callback: %s", esp_err_to_name(err));
        esp_now_deinit();
        return err;
    }

    err = esp_now_register_recv_cb(espnow_recv_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register recv callback: %s", esp_err_to_name(err));
        esp_now_deinit();
        return err;
    }

    // Add broadcast peer for sending to all
    esp_now_peer_info_t broadcast_peer = {
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(broadcast_peer.peer_addr, s_broadcast_mac, 6);

    err = esp_now_add_peer(&broadcast_peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(err));
        esp_now_deinit();
        return err;
    }

    // Add configured peers
    add_peer_if_valid(s_master_mac, "Master");
    add_peer_if_valid(s_rudder_mac, "Rudder");
    add_peer_if_valid(s_ui_mac, "UI");

    s_initialized = true;
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}

void espnow_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    esp_now_deinit();
    s_initialized = false;
    s_recv_callback = NULL;

    ESP_LOGI(TAG, "ESP-NOW deinitialized");
}

esp_err_t espnow_send(uint8_t msg_type, const uint8_t *data, size_t len,
                       const uint8_t *dest_mac)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (len > ESPNOW_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too large: %d > %d", (int)len, ESPNOW_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }

    // Build frame
    espnow_frame_t frame;
    frame.msg_type = msg_type;
    memset(frame.data, 0, ESPNOW_MAX_PAYLOAD);
    if (data && len > 0) {
        memcpy(frame.data, data, len);
    }

    // Use broadcast if no dest specified
    const uint8_t *target_mac = dest_mac ? dest_mac : s_broadcast_mac;

    // Check if peer exists, add if not
    if (!esp_now_is_peer_exist(target_mac)) {
        esp_now_peer_info_t peer = {
            .channel = 0,
            .ifidx = WIFI_IF_STA,
            .encrypt = false,
        };
        memcpy(peer.peer_addr, target_mac, 6);
        esp_err_t err = esp_now_add_peer(&peer);
        if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
            ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(err));
            return err;
        }
    }

    // Send
    esp_err_t err = esp_now_send(target_mac, (const uint8_t*)&frame, 1 + len);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "esp_now_send failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void espnow_register_recv_callback(espnow_recv_cb_t callback)
{
    s_recv_callback = callback;
}

const uint8_t* espnow_get_master_mac(void)
{
    return s_master_mac;
}

const uint8_t* espnow_get_rudder_mac(void)
{
    return s_rudder_mac;
}

const uint8_t* espnow_get_ui_mac(void)
{
    return s_ui_mac;
}

void espnow_get_status(espnow_status_t *status)
{
    if (!status) return;

    memset(status, 0, sizeof(*status));
    status->initialized = s_initialized;

    // Get current channel
    uint8_t primary;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) {
        status->channel = primary;
    }

    // Count peers
    esp_now_peer_num_t peer_num;
    if (esp_now_get_peer_num(&peer_num) == ESP_OK) {
        status->peer_count = peer_num.total_num;
    }

    status->tx_success_count = s_tx_success_count;
    status->tx_fail_count = s_tx_fail_count;
    status->rx_count = s_rx_count;
    status->rx_error_count = s_rx_error_count;

    memcpy(status->own_mac, s_own_mac, 6);
}

bool espnow_peer_exists(const uint8_t *mac)
{
    if (!mac) return false;
    return esp_now_is_peer_exist(mac);
}
