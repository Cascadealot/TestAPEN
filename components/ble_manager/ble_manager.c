/**
 * @file ble_manager.c
 * @brief BLE Manager Implementation for TestAPEN
 *
 * Implements NimBLE GATT service with 5 characteristics:
 * - Command (Write): Accept commands from BLE client
 * - Status (Read/Notify): System state and faults
 * - Heading (Read/Notify): Current and target heading
 * - Rudder (Read/Notify): Rudder angle and motor status
 * - Parameters (Read/Write/Notify): Configurable parameters
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 11
 */

#include "ble_manager.h"
#include "param_store.h"
#include "state_machine.h"
#include "icm20948.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>

static const char *TAG = "BLE";

/*============================================================================
 * Forward Declarations (Master Node Interface)
 *============================================================================*/

extern system_state_t master_get_state(void);
extern uint8_t master_get_fault_code(void);
extern bool master_engage(void);
extern bool master_disengage(void);
extern bool master_clear_fault(void);
extern float master_get_heading_filtered(void);
extern float master_get_target_heading(void);
extern void master_set_target_heading(float heading);
extern void master_adjust_target_heading(float delta);
extern bool master_enter_calibration(void);
extern bool master_exit_calibration(void);
extern bool master_calibrate_center(void);
extern bool master_calibrate_port(void);
extern bool master_calibrate_starboard(void);
extern bool master_calibrate_save(void);

/*============================================================================
 * UUIDs
 *============================================================================*/

// Service UUID: 12345678-1234-1234-1234-123456789abc
static const ble_uuid128_t s_service_uuid =
    BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Characteristic UUIDs: ...9001 through ...9005
static const ble_uuid128_t s_command_uuid =
    BLE_UUID128_INIT(0x01, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_status_uuid =
    BLE_UUID128_INIT(0x02, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_heading_uuid =
    BLE_UUID128_INIT(0x03, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_rudder_uuid =
    BLE_UUID128_INIT(0x04, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_params_uuid =
    BLE_UUID128_INIT(0x05, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_magcal_uuid =
    BLE_UUID128_INIT(0x06, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                     0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/*============================================================================
 * State
 *============================================================================*/

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_connected = false;

// Attribute handles for notifications
static uint16_t s_status_attr_handle;
static uint16_t s_heading_attr_handle;
static uint16_t s_rudder_attr_handle;
static uint16_t s_params_attr_handle;
static uint16_t s_magcal_attr_handle;

// Current values for characteristics
static uint8_t s_status_value[4] = {0};    // state, fault, flags, reserved
static uint8_t s_heading_value[4] = {0};   // current_x10 (2B), target_x10 (2B)
static uint8_t s_rudder_value[3] = {0};    // angle_x10 (2B), motor_status (1B)
static uint8_t s_params_value[13] = {0};   // id, value(4B), min(4B), max(4B)
static uint8_t s_magcal_value[49] = {0};   // cmd(1B), hard_iron(12B), soft_iron(36B)

/*============================================================================
 * GATT Access Callbacks
 *============================================================================*/

static int on_command_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int on_status_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int on_heading_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);
static int on_rudder_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int on_params_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);
static int on_magcal_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

/*============================================================================
 * GATT Service Definition
 *============================================================================*/

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // Command characteristic (Write)
                .uuid = &s_command_uuid.u,
                .access_cb = on_command_access,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // Status characteristic (Read/Notify)
                .uuid = &s_status_uuid.u,
                .access_cb = on_status_access,
                .val_handle = &s_status_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Heading characteristic (Read/Notify)
                .uuid = &s_heading_uuid.u,
                .access_cb = on_heading_access,
                .val_handle = &s_heading_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Rudder characteristic (Read/Notify)
                .uuid = &s_rudder_uuid.u,
                .access_cb = on_rudder_access,
                .val_handle = &s_rudder_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Parameters characteristic (Read/Write/Notify)
                .uuid = &s_params_uuid.u,
                .access_cb = on_params_access,
                .val_handle = &s_params_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                // Mag Calibration characteristic (Read/Write)
                .uuid = &s_magcal_uuid.u,
                .access_cb = on_magcal_access,
                .val_handle = &s_magcal_attr_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            { 0 }, // End of characteristics
        },
    },
    { 0 }, // End of services
};

/*============================================================================
 * Command Processing
 *============================================================================*/

static void process_command(const uint8_t *data, uint16_t len) {
    if (len < 1) return;

    uint8_t cmd = data[0];
    ESP_LOGI(TAG, "BLE command: 0x%02X", cmd);

    switch (cmd) {
        case BLE_CMD_ENGAGE:
            master_engage();
            break;

        case BLE_CMD_DISENGAGE:
            master_disengage();
            break;

        case BLE_CMD_SET_HEADING:
            if (len >= 3) {
                // Big-endian format: MSB first (matches Android app)
                int16_t heading_x10 = (int16_t)((data[1] << 8) | data[2]);
                float heading = heading_x10 / 10.0f;
                master_set_target_heading(heading);
                ESP_LOGI(TAG, "Set heading: %.1f", heading);
            }
            break;

        case BLE_CMD_ADJUST_HEADING:
            if (len >= 2) {
                int8_t delta = (int8_t)data[1];
                master_adjust_target_heading((float)delta);
                ESP_LOGI(TAG, "Adjust heading: %d", delta);
            }
            break;

        case BLE_CMD_CAL_ENTER:
            master_enter_calibration();
            break;

        case BLE_CMD_CAL_EXIT:
            master_exit_calibration();
            break;

        case BLE_CMD_CAL_CENTER:
            master_calibrate_center();
            break;

        case BLE_CMD_CAL_PORT:
            master_calibrate_port();
            break;

        case BLE_CMD_CAL_STBD:
            master_calibrate_starboard();
            break;

        case BLE_CMD_CAL_SAVE:
            master_calibrate_save();
            break;

        case BLE_CMD_FAULT_CLEAR:
            master_clear_fault();
            break;

        case BLE_CMD_STATUS_REQ:
            // NOTE: Do NOT send notifications from GATT callback context!
            // This causes crashes due to NimBLE reentrancy issues.
            // The master node sends periodic updates via ble_manager_update_*()
            // from its main task - those are safe.
            ESP_LOGI(TAG, "STATUS_REQ received (updates sent by periodic task)");
            break;

        case BLE_CMD_PARAM_GET:
            if (len >= 2) {
                param_id_t id = (param_id_t)data[1];
                if (id < PARAM_COUNT) {
                    const param_meta_t *meta = param_get_meta(id);
                    if (meta == NULL) {
                        ESP_LOGW(TAG, "PARAM_GET: invalid param id %d", id);
                        break;
                    }
                    float value = param_get(id);
                    ESP_LOGI(TAG, "PARAM_GET: id=%d, value=%.3f", id, value);

                    // Build response in params_value for READ access
                    s_params_value[0] = id;
                    memcpy(&s_params_value[1], &value, 4);
                    memcpy(&s_params_value[5], &meta->min_value, 4);
                    memcpy(&s_params_value[9], &meta->max_value, 4);

                    // NOTE: Do NOT send notifications from callback context!
                    // The app should READ the params characteristic to get the value.
                }
            }
            break;

        case BLE_CMD_PARAM_SET:
            if (len >= 6) {
                param_id_t id = (param_id_t)data[1];
                float value;
                memcpy(&value, &data[2], 4);

                esp_err_t err = param_set(id, value);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Set param %d = %.3f", id, value);
                } else {
                    ESP_LOGW(TAG, "Failed to set param %d", id);
                }
            }
            break;

        case BLE_CMD_PARAM_SAVE:
            if (len >= 2) {
                uint8_t id = data[1];
                if (id == 0xFF) {
                    param_save_all();
                    ESP_LOGI(TAG, "Saved all params");
                } else if (id < PARAM_COUNT) {
                    param_save((param_id_t)id);
                    ESP_LOGI(TAG, "Saved param %d", id);
                }
            }
            break;

        case BLE_CMD_PARAM_RESET:
            param_reset_defaults();
            ESP_LOGI(TAG, "Reset all params to defaults");
            break;

        default:
            ESP_LOGW(TAG, "Unknown BLE command: 0x%02X", cmd);
            break;
    }
}

/*============================================================================
 * GATT Access Callback Implementations
 *============================================================================*/

static int on_command_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t buf[20];
        if (len > sizeof(buf)) len = sizeof(buf);
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);
        process_command(buf, len);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int on_status_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Update status before read
        s_status_value[0] = (uint8_t)master_get_state();
        s_status_value[1] = master_get_fault_code();
        s_status_value[2] = 0; // flags
        s_status_value[3] = 0; // reserved

        int rc = os_mbuf_append(ctxt->om, s_status_value, sizeof(s_status_value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int on_heading_access(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Update heading before read
        int16_t current_x10 = (int16_t)(master_get_heading_filtered() * 10);
        int16_t target_x10 = (int16_t)(master_get_target_heading() * 10);
        memcpy(&s_heading_value[0], &current_x10, 2);
        memcpy(&s_heading_value[2], &target_x10, 2);

        int rc = os_mbuf_append(ctxt->om, s_heading_value, sizeof(s_heading_value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int on_rudder_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, s_rudder_value, sizeof(s_rudder_value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int on_params_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        int rc = os_mbuf_append(ctxt->om, s_params_value, sizeof(s_params_value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Write to params is same as PARAM_SET command
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t buf[20];
        if (len > sizeof(buf)) len = sizeof(buf);
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);

        if (len >= 5) {
            param_id_t id = (param_id_t)buf[0];
            float value;
            memcpy(&value, &buf[1], 4);
            param_set(id, value);
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int on_magcal_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Return current calibration values
        // For now just return zeros - full implementation would read from icm20948
        memset(s_magcal_value, 0, sizeof(s_magcal_value));
        int rc = os_mbuf_append(ctxt->om, s_magcal_value, sizeof(s_magcal_value));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        uint8_t buf[64];  // Max expected: 49 bytes
        if (len > sizeof(buf)) len = sizeof(buf);
        ble_hs_mbuf_to_flat(ctxt->om, buf, len, NULL);

        if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        uint8_t cmd = buf[0];
        ESP_LOGI(TAG, "MagCal command: 0x%02X, len=%d", cmd, len);

        switch (cmd) {
            case BLE_CMD_MAGCAL_SET:
                // Expect 49 bytes: cmd(1) + hard_iron(12) + soft_iron(36)
                if (len >= 49) {
                    float hx, hy, hz;
                    memcpy(&hx, &buf[1], 4);
                    memcpy(&hy, &buf[5], 4);
                    memcpy(&hz, &buf[9], 4);
                    icm20948_set_mag_cal(hx, hy, hz);

                    float soft[9];
                    memcpy(soft, &buf[13], 36);
                    icm20948_set_mag_soft_iron(soft);

                    ESP_LOGI(TAG, "MagCal SET: hard=(%.2f,%.2f,%.2f)", hx, hy, hz);
                } else {
                    ESP_LOGW(TAG, "MagCal SET: invalid length %d (expected 49)", len);
                    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
                }
                break;

            case BLE_CMD_MAGCAL_SAVE:
                // Save current calibration to NVS
                if (icm20948_save_mag_cal_to_nvs() == ESP_OK) {
                    ESP_LOGI(TAG, "MagCal saved to NVS");
                } else {
                    ESP_LOGE(TAG, "MagCal save failed");
                }
                break;

            case BLE_CMD_MAGCAL_RESET:
                // Reset to defaults (identity matrix, zero offsets)
                icm20948_reset_mag_cal();
                ESP_LOGI(TAG, "MagCal reset to defaults");
                break;

            default:
                ESP_LOGW(TAG, "Unknown MagCal command: 0x%02X", cmd);
                break;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/*============================================================================
 * GAP Event Handler
 *============================================================================*/

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_connected = true;
                ESP_LOGI(TAG, "BLE connected, handle=%d", s_conn_handle);
            } else {
                // Connection failed, restart advertising
                ESP_LOGW(TAG, "BLE connect failed, status=%d", event->connect.status);
                ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                  NULL, ble_gap_event, NULL);
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_connected = false;
            ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
            // Restart advertising
            ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                              NULL, ble_gap_event, NULL);
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            break;

        default:
            break;
    }
    return 0;
}

/*============================================================================
 * BLE Host Task
 *============================================================================*/

static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void on_sync(void) {
    ESP_LOGI(TAG, "BLE stack synced");

    // Set up advertising data
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"CascadeAP";
    fields.name_len = 9;
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data: %d", rc);
        return;
    }

    // Set up scan response with service UUID
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128 = &s_service_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response: %d", rc);
        // Continue anyway - advertising will still work
    }

    // Start advertising
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 160,  // 100ms
        .itvl_max = 320,  // 200ms
    };

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started as 'CascadeAP'");
    }
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "BLE reset, reason=%d", reason);
}

/*============================================================================
 * Public API
 *============================================================================*/

esp_err_t ble_manager_init(void) {
    ESP_LOGI(TAG, "Initializing BLE manager");

    // Initialize NVS (required by NimBLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure host callbacks
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Set device name
    ble_svc_gap_device_name_set("CascadeAP");

    // Initialize GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT count failed: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT add services failed: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "GATT services registered");

    // NOTE: Do NOT call ble_gatts_start() here - NimBLE host task
    // calls it automatically during ble_hs_start(). Calling it twice
    // corrupts the GATT server and causes crashes!

    // Start host task - this will call ble_hs_start() which calls ble_gatts_start()
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE manager initialized");
    return ESP_OK;
}

bool ble_manager_is_connected(void) {
    return s_connected;
}

void ble_manager_update_status(uint8_t state, uint8_t fault, uint8_t flags) {
    s_status_value[0] = state;
    s_status_value[1] = fault;
    s_status_value[2] = flags;
    s_status_value[3] = 0;

    if (s_connected) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_status_value, sizeof(s_status_value));
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_status_attr_handle, om);
        }
    }
}

void ble_manager_update_heading(float current, float target) {
    int16_t current_x10 = (int16_t)(current * 10);
    int16_t target_x10 = (int16_t)(target * 10);
    memcpy(&s_heading_value[0], &current_x10, 2);
    memcpy(&s_heading_value[2], &target_x10, 2);

    if (s_connected) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_heading_value, sizeof(s_heading_value));
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_heading_attr_handle, om);
        }
    }
}

void ble_manager_update_rudder(float angle, uint8_t motor_status) {
    int16_t angle_x10 = (int16_t)(angle * 10);
    memcpy(&s_rudder_value[0], &angle_x10, 2);
    s_rudder_value[2] = motor_status;

    if (s_connected) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_rudder_value, sizeof(s_rudder_value));
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_rudder_attr_handle, om);
        }
    }
}

void ble_manager_notify_param(uint8_t param_id, float value) {
    if (param_id >= PARAM_COUNT) return;

    const param_meta_t *meta = param_get_meta((param_id_t)param_id);
    if (!meta) return;

    s_params_value[0] = param_id;
    memcpy(&s_params_value[1], &value, 4);
    memcpy(&s_params_value[5], &meta->min_value, 4);
    memcpy(&s_params_value[9], &meta->max_value, 4);

    if (s_connected) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_params_value, sizeof(s_params_value));
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_params_attr_handle, om);
        }
    }
}
