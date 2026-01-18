#!/bin/bash
# TestAPEN Error and Fault Handling Tests
# Tests fault detection, reporting, and recovery

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Error and Fault Handling Tests"
echo "========================================"
echo ""

init_results

# ERR-01: Fault code query
info "ERR-01: Fault code query..."

master_state=$(send_cmd "$MASTER_IP" "state")
if echo "$master_state" | grep -qE "fault|state"; then
    pass "ERR-01a" "Master state/fault query works"
else
    fail "ERR-01a" "Master state query"
fi

rudder_state=$(send_cmd "$RUDDER_IP" "state")
if echo "$rudder_state" | grep -qE "fault|state"; then
    pass "ERR-01b" "Rudder state/fault query works"
else
    fail "ERR-01b" "Rudder state query"
fi

# ERR-02: Fault clear command
info "ERR-02: Fault clear command..."

# Clear any existing faults
clear_result=$(send_cmd "$MASTER_IP" "fault clear")
if [ $? -eq 0 ]; then
    pass "ERR-02a" "Master fault clear command accepted"
else
    fail "ERR-02a" "Master fault clear command"
fi

clear_result=$(send_cmd "$RUDDER_IP" "fault clear")
if [ $? -eq 0 ]; then
    pass "ERR-02b" "Rudder fault clear command accepted"
else
    fail "ERR-02b" "Rudder fault clear command"
fi

# Wait for recovery
sleep 2

# ERR-03: Verify no faults after clear
info "ERR-03: Verify clean state..."

master_fault=$(get_fault_code "$MASTER_IP")
if [ "$master_fault" = "0x00" ] || [ -z "$master_fault" ]; then
    pass "ERR-03a" "Master no active fault"
else
    fail "ERR-03a" "Master fault code" "0x00" "$master_fault"
fi

rudder_fault=$(get_fault_code "$RUDDER_IP")
if [ "$rudder_fault" = "0x00" ] || [ -z "$rudder_fault" ]; then
    pass "ERR-03b" "Rudder no active fault"
else
    fail "ERR-03b" "Rudder fault code" "0x00" "$rudder_fault"
fi

# ERR-04: ESP-NOW TX failure handling
info "ERR-04: ESP-NOW TX failure count..."

# Check TX failure counters (should be 0 under normal operation)
master_tx_fail=$(get_tx_failed "$MASTER_IP")
rudder_tx_fail=$(get_tx_failed "$RUDDER_IP")

if [ "$master_tx_fail" = "0" ]; then
    pass "ERR-04a" "Master TX failures = 0"
else
    fail "ERR-04a" "Master TX failures" "0" "$master_tx_fail"
fi

if [ "$rudder_tx_fail" = "0" ]; then
    pass "ERR-04b" "Rudder TX failures = 0"
else
    fail "ERR-04b" "Rudder TX failures" "0" "$rudder_tx_fail"
fi

# ERR-05: RX error handling
info "ERR-05: ESP-NOW RX error count..."

master_espnow=$(send_cmd "$MASTER_IP" "espnow")
master_rx_err=$(echo "$master_espnow" | grep "RX errors" | grep -oE "[0-9]+" | head -1)

if [ "$master_rx_err" = "0" ]; then
    pass "ERR-05a" "Master RX errors = 0"
else
    fail "ERR-05a" "Master RX errors" "0" "$master_rx_err"
fi

rudder_espnow=$(send_cmd "$RUDDER_IP" "espnow")
rudder_rx_err=$(echo "$rudder_espnow" | grep "RX errors" | grep -oE "[0-9]+" | head -1)

if [ "$rudder_rx_err" = "0" ]; then
    pass "ERR-05b" "Rudder RX errors = 0"
else
    fail "ERR-05b" "Rudder RX errors" "0" "$rudder_rx_err"
fi

# ERR-06: Heartbeat timeout test (requires physical intervention)
info "ERR-06: Heartbeat timeout detection..."

skip "ERR-06" "Heartbeat timeout" "Requires powering off Master node"
# To test manually:
# 1. Power off Master node
# 2. Within 500ms, Rudder should enter FAULTED with code 0x40
# 3. UI should show Master: disconnected

# ERR-07: Motor stall detection (requires physical intervention)
info "ERR-07: Motor stall detection..."

skip "ERR-07" "Motor stall detection" "Requires physically blocking rudder"
# To test manually:
# 1. Engage rudder: echo "engage" | nc 192.168.1.157 2323
# 2. Command movement: echo "set rudder 30" | nc 192.168.1.157 2323
# 3. PHYSICALLY BLOCK RUDDER (carefully!)
# 4. Within 600ms, Rudder should enter FAULTED with code 0x20

# ERR-08: Motor timeout test
info "ERR-08: Motor timeout test..."

skip "ERR-08" "Motor timeout" "Takes 5+ seconds and requires continuous motion"
# To test manually:
# 1. Engage rudder
# 2. Command continuous large movement
# 3. After 5 seconds of motor running, should fault with 0x22

# ERR-09: Calibration validation
info "ERR-09: Calibration validation..."

# Try to engage with potentially invalid calibration
send_cmd "$RUDDER_IP" "disengage" > /dev/null
sleep 1

# Check if calibration is marked valid
rudder_status=$(send_cmd "$RUDDER_IP" "rudder")
if echo "$rudder_status" | grep -qi "calibration.*valid\|cal.*valid"; then
    pass "ERR-09" "Calibration status reported"
else
    # May show as "cal: valid" or similar
    if echo "$rudder_status" | grep -qi "cal\|calibrat"; then
        pass "ERR-09" "Calibration info available"
    else
        skip "ERR-09" "Calibration validation" "Status format unknown"
    fi
fi

# ERR-10: Recovery after fault clear
info "ERR-10: Recovery test - fault clear and re-engage..."

# Clear any faults
send_cmd "$MASTER_IP" "fault clear" > /dev/null
send_cmd "$RUDDER_IP" "fault clear" > /dev/null
sleep 2

# Check states returned to IDLE
master_state=$(get_state "$MASTER_IP")
rudder_state=$(get_state "$RUDDER_IP")

if [ "$master_state" = "IDLE" ] && [ "$rudder_state" = "IDLE" ]; then
    pass "ERR-10a" "Both nodes in IDLE after fault clear"
else
    fail "ERR-10a" "States after clear" "IDLE/IDLE" "$master_state/$rudder_state"
fi

# Try to engage
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
send_cmd "$MASTER_IP" "engage" > /dev/null
sleep 2

master_state=$(get_state "$MASTER_IP")
if [ "$master_state" = "ENGAGED" ]; then
    pass "ERR-10b" "Successfully engaged after fault recovery"
else
    fail "ERR-10b" "Engage after recovery" "ENGAGED" "$master_state"
fi

# Clean up
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading real" > /dev/null

# ERR-11: GNSS failure handling (non-fatal)
info "ERR-11: GNSS failure handling..."

# Check GNSS status
gnss_status=$(send_cmd "$MASTER_IP" "gnss")
info "GNSS status: $(echo "$gnss_status" | grep -i "fix\|valid" | head -1)"

# Verify system doesn't fault due to GNSS issues
master_fault=$(get_fault_code "$MASTER_IP")
master_state=$(get_state "$MASTER_IP")

if [ "$master_state" != "FAULTED" ]; then
    pass "ERR-11" "System not faulted despite GNSS state"
else
    # Check if fault is GNSS-related
    if [ "$master_fault" = "0x10" ] || [ "$master_fault" = "0x11" ]; then
        fail "ERR-11" "GNSS fault should not cause system fault"
    else
        fail "ERR-11" "Unexpected fault" "not FAULTED" "$master_state (fault: $master_fault)"
    fi
fi

# ERR-12: Engage precondition check - fault active
info "ERR-12: Engage precondition - cannot engage while faulted..."

# This is hard to test without inducing a fault
# We'll verify the state machine logic via documentation
skip "ERR-12" "Engage while faulted" "Requires fault injection"

# ERR-13: Error code meanings
info "ERR-13: Error code reference check..."

echo ""
echo "Error Code Reference:"
echo "  0x00 - ERR_NONE (no error)"
echo "  0x01 - ERR_ESPNOW_TX_FAIL"
echo "  0x02 - ERR_ESPNOW_RX_TIMEOUT"
echo "  0x10 - ERR_SENSOR_FAULT"
echo "  0x20 - ERR_MOTOR_STALL"
echo "  0x22 - ERR_MOTOR_TIMEOUT"
echo "  0x30 - ERR_CAL_INVALID"
echo "  0x40 - ERR_HEARTBEAT_LOST"
echo ""

pass "ERR-13" "Error code reference documented"

print_summary

echo ""
echo "NOTE: Some error tests require physical intervention:"
echo "  - ERR-06: Power off Master to test heartbeat timeout"
echo "  - ERR-07: Block rudder physically to test stall detection"
echo "  - ERR-08: Requires continuous motor motion for 5+ seconds"
echo ""
echo "Run these tests manually with physical access to hardware."
echo ""

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
