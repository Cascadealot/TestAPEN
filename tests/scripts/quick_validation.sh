#!/bin/bash
# TestAPEN Quick Validation Tests
# Run these tests first to verify basic system operation

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Quick Validation Tests"
echo "========================================"
echo ""

init_results

# QV-01: Check all nodes online
info "QV-01: Checking all nodes online..."

if check_node "$MASTER_IP" "Master"; then
    pass "QV-01a" "Master node online"
else
    fail "QV-01a" "Master node online" "reachable" "unreachable"
fi

if check_node "$RUDDER_IP" "Rudder"; then
    pass "QV-01b" "Rudder node online"
else
    fail "QV-01b" "Rudder node online" "reachable" "unreachable"
fi

if check_node "$UI_IP" "UI"; then
    pass "QV-01c" "UI node online"
else
    fail "QV-01c" "UI node online" "reachable" "unreachable"
fi

# QV-02: Check version strings
info "QV-02: Checking firmware versions..."

master_ver=$(send_cmd "$MASTER_IP" "version" | grep -i "version" | head -1)
if echo "$master_ver" | grep -q "espnow"; then
    pass "QV-02a" "Master firmware version contains 'espnow'"
else
    fail "QV-02a" "Master firmware version" "contains 'espnow'" "$master_ver"
fi

rudder_ver=$(send_cmd "$RUDDER_IP" "version" | grep -i "version" | head -1)
if echo "$rudder_ver" | grep -q "espnow"; then
    pass "QV-02b" "Rudder firmware version contains 'espnow'"
else
    fail "QV-02b" "Rudder firmware version" "contains 'espnow'" "$rudder_ver"
fi

ui_ver=$(send_cmd "$UI_IP" "version" | grep -i "version" | head -1)
if echo "$ui_ver" | grep -q "espnow"; then
    pass "QV-02c" "UI firmware version contains 'espnow'"
else
    fail "QV-02c" "UI firmware version" "contains 'espnow'" "$ui_ver"
fi

# QV-03: Check ESP-NOW status
info "QV-03: Checking ESP-NOW communication..."

# Check Master ESP-NOW
master_espnow=$(send_cmd "$MASTER_IP" "espnow")
if echo "$master_espnow" | grep -q "Initialized: yes"; then
    pass "QV-03a" "Master ESP-NOW initialized"
else
    fail "QV-03a" "Master ESP-NOW initialized"
fi

master_tx_fail=$(echo "$master_espnow" | grep "TX failed" | grep -oE "[0-9]+" | head -1)
if [ "$master_tx_fail" = "0" ]; then
    pass "QV-03b" "Master TX failures = 0"
else
    fail "QV-03b" "Master TX failures" "0" "$master_tx_fail"
fi

# Check Rudder ESP-NOW
rudder_espnow=$(send_cmd "$RUDDER_IP" "espnow")
if echo "$rudder_espnow" | grep -q "Initialized: yes"; then
    pass "QV-03c" "Rudder ESP-NOW initialized"
else
    fail "QV-03c" "Rudder ESP-NOW initialized"
fi

rudder_rx=$(echo "$rudder_espnow" | grep "RX count" | grep -oE "[0-9]+" | head -1)
if [ -n "$rudder_rx" ] && [ "$rudder_rx" -gt "0" ]; then
    pass "QV-03d" "Rudder receiving messages (RX: $rudder_rx)"
else
    fail "QV-03d" "Rudder receiving messages" ">0" "$rudder_rx"
fi

# Check UI ESP-NOW
ui_espnow=$(send_cmd "$UI_IP" "espnow")
if echo "$ui_espnow" | grep -q "Initialized: yes"; then
    pass "QV-03e" "UI ESP-NOW initialized"
else
    fail "QV-03e" "UI ESP-NOW initialized"
fi

ui_rx=$(echo "$ui_espnow" | grep "RX count" | grep -oE "[0-9]+" | head -1)
if [ -n "$ui_rx" ] && [ "$ui_rx" -gt "0" ]; then
    pass "QV-03f" "UI receiving messages (RX: $ui_rx)"
else
    fail "QV-03f" "UI receiving messages" ">0" "$ui_rx"
fi

# QV-04: Check system states
info "QV-04: Checking system states..."

assert_state "$MASTER_IP" "IDLE" "QV-04a" "Master in IDLE state"
assert_state "$RUDDER_IP" "IDLE" "QV-04b" "Rudder in IDLE state"

# QV-05: Check no faults
info "QV-05: Checking for faults..."

master_fault=$(get_fault_code "$MASTER_IP")
if [ "$master_fault" = "0x00" ] || [ -z "$master_fault" ]; then
    pass "QV-05a" "Master no active fault"
else
    fail "QV-05a" "Master fault code" "0x00" "$master_fault"
fi

rudder_fault=$(get_fault_code "$RUDDER_IP")
if [ "$rudder_fault" = "0x00" ] || [ -z "$rudder_fault" ]; then
    pass "QV-05b" "Rudder no active fault"
else
    fail "QV-05b" "Rudder fault code" "0x00" "$rudder_fault"
fi

# QV-06: Check UI node connectivity display
info "QV-06: Checking UI connectivity status..."

ui_status=$(send_cmd "$UI_IP" "status")
if echo "$ui_status" | grep -qi "Master.*Connected"; then
    pass "QV-06a" "UI shows Master connected"
else
    fail "QV-06a" "UI shows Master connected"
fi

if echo "$ui_status" | grep -qi "Rudder.*Connected"; then
    pass "QV-06b" "UI shows Rudder connected"
else
    fail "QV-06b" "UI shows Rudder connected"
fi

# QV-07: Check MAC address configuration
info "QV-07: Checking MAC address configuration..."

ui_peers=$(send_cmd "$UI_IP" "espnow" | grep -A5 "Configured Peers")
if echo "$ui_peers" | grep -q "78:42:1C:6C:FA:58"; then
    pass "QV-07a" "Master MAC correctly configured"
else
    fail "QV-07a" "Master MAC configuration"
fi

if echo "$ui_peers" | grep -q "78:42:1C:6D:28:94"; then
    pass "QV-07b" "Rudder MAC correctly configured"
else
    fail "QV-07b" "Rudder MAC configuration"
fi

if echo "$ui_peers" | grep -q "78:42:1C:6B:E5:F0"; then
    pass "QV-07c" "UI MAC correctly configured"
else
    fail "QV-07c" "UI MAC configuration"
fi

# QV-08: WiFi channel match
info "QV-08: Checking WiFi channel..."

master_ch=$(send_cmd "$MASTER_IP" "espnow" | grep "Channel" | grep -oE "[0-9]+" | head -1)
rudder_ch=$(send_cmd "$RUDDER_IP" "espnow" | grep "Channel" | grep -oE "[0-9]+" | head -1)
ui_ch=$(send_cmd "$UI_IP" "espnow" | grep "Channel" | grep -oE "[0-9]+" | head -1)

if [ "$master_ch" = "$rudder_ch" ] && [ "$rudder_ch" = "$ui_ch" ]; then
    pass "QV-08" "All nodes on same WiFi channel ($master_ch)"
else
    fail "QV-08" "WiFi channel match" "all same" "M:$master_ch R:$rudder_ch U:$ui_ch"
fi

print_summary

# Exit with failure code if any tests failed
if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
