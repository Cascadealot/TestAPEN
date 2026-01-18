/**
 * @file state_machine.c
 * @brief State Machine Implementation for TestAP2 Autopilot
 *
 * FSD Reference: TestAP2.FSD.v1.0.0.md Section 8
 */

#include "state_machine.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "STATE";

void state_machine_init(state_machine_t *sm) {
    sm->current_state = STATE_BOOT;
    sm->previous_state = STATE_BOOT;
    sm->fault_code = 0;
    sm->state_entry_time_ms = esp_timer_get_time() / 1000;
    sm->motor_enabled = false;

    ESP_LOGI(TAG, "State machine initialized in BOOT state");
}

static bool check_engage_preconditions(const engage_preconditions_t *pre) {
    if (!pre) return false;

    bool ok = pre->heading_valid &&
              pre->rudder_feedback_valid &&
              pre->calibration_valid &&
              pre->no_active_faults;

    if (!ok) {
        ESP_LOGW(TAG, "Engage preconditions not met: heading=%d, rudder=%d, cal=%d, faults=%d",
                 pre->heading_valid, pre->rudder_feedback_valid,
                 pre->calibration_valid, pre->no_active_faults);
    }

    return ok;
}

static void enter_state(state_machine_t *sm, system_state_t new_state) {
    sm->previous_state = sm->current_state;
    sm->current_state = new_state;
    sm->state_entry_time_ms = esp_timer_get_time() / 1000;

    // Update motor_enabled based on state (FSD Section 8.5)
    switch (new_state) {
        case STATE_BOOT:
        case STATE_IDLE:
        case STATE_CALIBRATION:
        case STATE_FAULTED:
            sm->motor_enabled = false;
            break;
        case STATE_ENGAGED:
            sm->motor_enabled = true;
            break;
    }

    ESP_LOGI(TAG, "State transition: %s -> %s",
             state_to_string(sm->previous_state),
             state_to_string(new_state));
}

bool state_machine_process(state_machine_t *sm, state_event_t event,
                           const engage_preconditions_t *preconditions) {
    if (event == EVENT_NONE) return false;

    system_state_t current = sm->current_state;
    system_state_t next = current;

    // State transition logic (FSD Section 8.2)
    switch (current) {
        case STATE_BOOT:
            if (event == EVENT_POST_PASS) {
                next = STATE_IDLE;
            } else if (event == EVENT_POST_FAIL || event == EVENT_FAULT) {
                next = STATE_FAULTED;
            }
            break;

        case STATE_IDLE:
            if (event == EVENT_ENGAGE && check_engage_preconditions(preconditions)) {
                next = STATE_ENGAGED;
            } else if (event == EVENT_CAL_ENTER) {
                next = STATE_CALIBRATION;
            } else if (event == EVENT_FAULT) {
                next = STATE_FAULTED;
            }
            break;

        case STATE_ENGAGED:
            if (event == EVENT_DISENGAGE) {
                next = STATE_IDLE;
            } else if (event == EVENT_FAULT) {
                next = STATE_FAULTED;
            }
            break;

        case STATE_CALIBRATION:
            if (event == EVENT_CAL_EXIT) {
                next = STATE_IDLE;
            } else if (event == EVENT_FAULT) {
                next = STATE_FAULTED;
            }
            break;

        case STATE_FAULTED:
            if (event == EVENT_FAULT_CLEAR && sm->fault_code == 0) {
                next = STATE_IDLE;
            }
            break;
    }

    if (next != current) {
        enter_state(sm, next);
        return true;
    }

    return false;
}

system_state_t state_machine_get_state(const state_machine_t *sm) {
    return sm->current_state;
}

void state_machine_set_fault(state_machine_t *sm, uint8_t fault_code) {
    sm->fault_code = fault_code;
    ESP_LOGE(TAG, "Fault set: 0x%02X", fault_code);

    if (sm->current_state != STATE_FAULTED) {
        state_machine_process(sm, EVENT_FAULT, NULL);
    }
}

bool state_machine_motor_allowed(const state_machine_t *sm) {
    // Motor only allowed in ENGAGED state (FSD Section 8.5)
    return sm->current_state == STATE_ENGAGED;
}

const char *state_to_string(system_state_t state) {
    switch (state) {
        case STATE_BOOT:        return "BOOT";
        case STATE_IDLE:        return "IDLE";
        case STATE_ENGAGED:     return "ENGAGED";
        case STATE_CALIBRATION: return "CALIBRATION";
        case STATE_FAULTED:     return "FAULTED";
        default:                return "UNKNOWN";
    }
}
