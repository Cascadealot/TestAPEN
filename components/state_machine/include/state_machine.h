/**
 * @file state_machine.h
 * @brief State Machine for TestAPEN Autopilot
 *
 * FSD Reference: TestAPEN.FSD.v1.0.0.md Section 8
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * States (FSD Section 8.1)
 *============================================================================*/

typedef enum {
    STATE_BOOT          = 0x00,
    STATE_IDLE          = 0x01,
    STATE_ENGAGED       = 0x02,
    STATE_CALIBRATION   = 0x03,
    STATE_FAULTED       = 0xFF
} system_state_t;

/*============================================================================
 * State Events
 *============================================================================*/

typedef enum {
    EVENT_NONE,
    EVENT_POST_PASS,
    EVENT_POST_FAIL,
    EVENT_ENGAGE,
    EVENT_DISENGAGE,
    EVENT_CAL_ENTER,
    EVENT_CAL_EXIT,
    EVENT_FAULT,
    EVENT_FAULT_CLEAR
} state_event_t;

/*============================================================================
 * State Machine Context
 *============================================================================*/

typedef struct {
    system_state_t current_state;
    system_state_t previous_state;
    uint8_t fault_code;
    uint32_t state_entry_time_ms;
    bool motor_enabled;
} state_machine_t;

/*============================================================================
 * Engage Preconditions (FSD Section 8.3)
 *============================================================================*/

typedef struct {
    bool heading_valid;         // Not NaN, < 500ms old
    bool rudder_feedback_valid; // Heartbeat OK
    bool calibration_valid;     // Range >= 5°
    bool no_active_faults;      // fault_code == 0
} engage_preconditions_t;

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/**
 * @brief Initialize state machine
 * @param sm Pointer to state machine context
 */
void state_machine_init(state_machine_t *sm);

/**
 * @brief Process event and transition state
 * @param sm Pointer to state machine context
 * @param event Event to process
 * @param preconditions Engage preconditions (only needed for ENGAGE event)
 * @return true if transition occurred
 */
bool state_machine_process(state_machine_t *sm, state_event_t event,
                           const engage_preconditions_t *preconditions);

/**
 * @brief Get current state
 * @param sm Pointer to state machine context
 * @return Current state
 */
system_state_t state_machine_get_state(const state_machine_t *sm);

/**
 * @brief Set fault code and transition to FAULTED
 * @param sm Pointer to state machine context
 * @param fault_code Fault code to set
 */
void state_machine_set_fault(state_machine_t *sm, uint8_t fault_code);

/**
 * @brief Check if motor should be enabled in current state
 * @param sm Pointer to state machine context
 * @return true if motor should be enabled
 */
bool state_machine_motor_allowed(const state_machine_t *sm);

/**
 * @brief Get state name string
 * @param state State value
 * @return State name string
 */
const char *state_to_string(system_state_t state);

#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H
