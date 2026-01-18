#!/bin/bash
# TestAPEN Communication Tests
# Tests ESP-NOW message exchange between nodes

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Communication Tests"
echo "========================================"
echo ""

init_results

# COMM-01: Heartbeat rate test
info "COMM-01: Testing heartbeat rates..."

# Get initial RX counts
ui_rx_start=$(get_rx_count "$UI_IP")
rudder_rx_start=$(get_rx_count "$RUDDER_IP")

info "Initial counts - UI RX: $ui_rx_start, Rudder RX: $rudder_rx_start"
info "Waiting 5 seconds..."
sleep 5

# Get final RX counts
ui_rx_end=$(get_rx_count "$UI_IP")
rudder_rx_end=$(get_rx_count "$RUDDER_IP")

# Calculate messages received
ui_rx_diff=$((ui_rx_end - ui_rx_start))
rudder_rx_diff=$((rudder_rx_end - rudder_rx_start))

info "5-second counts - UI RX: +$ui_rx_diff, Rudder RX: +$rudder_rx_diff"

# Expected: Master @ 10Hz + Rudder @ 50Hz = 60Hz * 5s = 300 messages (approx)
# UI should receive both, Rudder receives Master only
# Allow 20% tolerance

if [ "$ui_rx_diff" -gt 200 ]; then
    pass "COMM-01a" "UI receiving heartbeats at expected rate (+$ui_rx_diff in 5s)"
else
    fail "COMM-01a" "UI heartbeat rate" ">200 in 5s" "$ui_rx_diff"
fi

# Rudder should receive ~50 Master heartbeats in 5s (10Hz * 5)
if [ "$rudder_rx_diff" -gt 30 ]; then
    pass "COMM-01b" "Rudder receiving Master heartbeats (+$rudder_rx_diff in 5s)"
else
    fail "COMM-01b" "Rudder heartbeat rate" ">30 in 5s" "$rudder_rx_diff"
fi

# COMM-02: TX success rate
info "COMM-02: Checking TX success rates..."

master_espnow=$(send_cmd "$MASTER_IP" "espnow")
master_tx_success=$(echo "$master_espnow" | grep "TX success" | grep -oE "[0-9]+" | head -1)
master_tx_fail=$(echo "$master_espnow" | grep "TX failed" | grep -oE "[0-9]+" | head -1)

if [ "$master_tx_fail" = "0" ]; then
    pass "COMM-02a" "Master TX failures = 0 (success: $master_tx_success)"
else
    fail "COMM-02a" "Master TX failures" "0" "$master_tx_fail"
fi

rudder_espnow=$(send_cmd "$RUDDER_IP" "espnow")
rudder_tx_success=$(echo "$rudder_espnow" | grep "TX success" | grep -oE "[0-9]+" | head -1)
rudder_tx_fail=$(echo "$rudder_espnow" | grep "TX failed" | grep -oE "[0-9]+" | head -1)

if [ "$rudder_tx_fail" = "0" ]; then
    pass "COMM-02b" "Rudder TX failures = 0 (success: $rudder_tx_success)"
else
    fail "COMM-02b" "Rudder TX failures" "0" "$rudder_tx_fail"
fi

# COMM-03: RX error rate
info "COMM-03: Checking RX error rates..."

master_rx_err=$(echo "$master_espnow" | grep "RX errors" | grep -oE "[0-9]+" | head -1)
if [ "$master_rx_err" = "0" ]; then
    pass "COMM-03a" "Master RX errors = 0"
else
    fail "COMM-03a" "Master RX errors" "0" "$master_rx_err"
fi

rudder_rx_err=$(echo "$rudder_espnow" | grep "RX errors" | grep -oE "[0-9]+" | head -1)
if [ "$rudder_rx_err" = "0" ]; then
    pass "COMM-03b" "Rudder RX errors = 0"
else
    fail "COMM-03b" "Rudder RX errors" "0" "$rudder_rx_err"
fi

ui_espnow=$(send_cmd "$UI_IP" "espnow")
ui_rx_err=$(echo "$ui_espnow" | grep "RX errors" | grep -oE "[0-9]+" | head -1)
if [ "$ui_rx_err" = "0" ]; then
    pass "COMM-03c" "UI RX errors = 0"
else
    fail "COMM-03c" "UI RX errors" "0" "$ui_rx_err"
fi

# COMM-04: Peer count
info "COMM-04: Checking peer counts..."

master_peers=$(echo "$master_espnow" | grep "Peers" | grep -oE "[0-9]+" | head -1)
if [ "$master_peers" -ge "2" ]; then
    pass "COMM-04a" "Master has $master_peers peers"
else
    fail "COMM-04a" "Master peer count" ">=2" "$master_peers"
fi

rudder_peers=$(echo "$rudder_espnow" | grep "Peers" | grep -oE "[0-9]+" | head -1)
if [ "$rudder_peers" -ge "2" ]; then
    pass "COMM-04b" "Rudder has $rudder_peers peers"
else
    fail "COMM-04b" "Rudder peer count" ">=2" "$rudder_peers"
fi

ui_peers=$(echo "$ui_espnow" | grep "Peers" | grep -oE "[0-9]+" | head -1)
if [ "$ui_peers" -ge "2" ]; then
    pass "COMM-04c" "UI has $ui_peers peers"
else
    fail "COMM-04c" "UI peer count" ">=2" "$ui_peers"
fi

# COMM-05: Command flow test (UI -> Master)
info "COMM-05: Testing command flow (UI -> Master)..."

# Get initial target heading
initial_target=$(get_target "$MASTER_IP")
info "Initial target heading: $initial_target"

# Record Master RX count
master_rx_before=$(get_rx_count "$MASTER_IP")

# Note: This test requires button press or we can test via console command on UI
# For automated testing, we'll skip the actual button press
skip "COMM-05" "UI to Master command flow" "Requires physical button press"

# COMM-06: Command flow test (Master -> Rudder)
info "COMM-06: Testing command flow (Master -> Rudder)..."

# Ensure system is in IDLE
send_cmd "$MASTER_IP" "disengage" > /dev/null

# Enable heading simulation
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
sleep 1

# Engage
send_cmd "$MASTER_IP" "engage" > /dev/null
sleep 1

# Check Rudder is receiving commands
rudder_status=$(send_cmd "$RUDDER_IP" "rudder")
if echo "$rudder_status" | grep -qE "commanded.*[0-9]"; then
    pass "COMM-06" "Rudder receiving commands from Master"
else
    fail "COMM-06" "Rudder command reception"
fi

# Disengage
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading real" > /dev/null

# COMM-07: State synchronization
info "COMM-07: Testing state synchronization..."

# Engage and check UI sees it
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
send_cmd "$MASTER_IP" "engage" > /dev/null
sleep 1

ui_status=$(send_cmd "$UI_IP" "status")
if echo "$ui_status" | grep -qi "ENGAGED"; then
    pass "COMM-07a" "UI shows ENGAGED state"
else
    fail "COMM-07a" "UI state sync" "shows ENGAGED" "not shown"
fi

# Disengage and check UI sees it
send_cmd "$MASTER_IP" "disengage" > /dev/null
sleep 1

ui_status=$(send_cmd "$UI_IP" "status")
if echo "$ui_status" | grep -qi "IDLE"; then
    pass "COMM-07b" "UI shows IDLE state"
else
    fail "COMM-07b" "UI state sync" "shows IDLE" "not shown"
fi

send_cmd "$MASTER_IP" "heading real" > /dev/null

# COMM-08: Heading data propagation
info "COMM-08: Testing heading data propagation..."

# Set simulated heading
send_cmd "$MASTER_IP" "heading sim 123" > /dev/null
sleep 2

# Check UI sees the heading
ui_status=$(send_cmd "$UI_IP" "status")
if echo "$ui_status" | grep -qE "Heading.*12[0-9]"; then
    pass "COMM-08" "UI receives heading data (~123 deg)"
else
    fail "COMM-08" "UI heading data" "~123 deg" "$(echo "$ui_status" | grep -i heading)"
fi

send_cmd "$MASTER_IP" "heading real" > /dev/null

# COMM-09: Rudder angle propagation
info "COMM-09: Testing rudder angle propagation..."

ui_status=$(send_cmd "$UI_IP" "status")
if echo "$ui_status" | grep -qE "Rudder.*[-]?[0-9]+\.[0-9]"; then
    pass "COMM-09" "UI receives rudder angle data"
else
    fail "COMM-09" "UI rudder data"
fi

# COMM-10: Latency test (rough estimate)
info "COMM-10: Latency test (rough estimate)..."

# This is a rough test - proper latency measurement requires timestamps
start_time=$(date +%s%N)
send_cmd "$MASTER_IP" "status" > /dev/null
end_time=$(date +%s%N)

latency_ms=$(( (end_time - start_time) / 1000000 ))
info "Console round-trip latency: ${latency_ms}ms"

if [ "$latency_ms" -lt 1000 ]; then
    pass "COMM-10" "Console latency < 1000ms ($latency_ms ms)"
else
    fail "COMM-10" "Console latency" "<1000ms" "${latency_ms}ms"
fi

print_summary

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
