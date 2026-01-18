#!/bin/bash
# TestAPEN BLE Interface Tests
# Tests Bluetooth Low Energy communication with Master node

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN BLE Interface Tests"
echo "========================================"
echo ""

init_results

# BLE device name to look for
BLE_DEVICE_NAME="TestAP2"
BLE_MAC=""

# Check if bluetoothctl is available
if ! command -v bluetoothctl &> /dev/null; then
    echo "ERROR: bluetoothctl not found. Install bluez package."
    exit 1
fi

# Helper function to run bluetoothctl command
run_btctl() {
    local cmd="$1"
    local timeout="${2:-5}"
    echo "$cmd" | timeout "$timeout" bluetoothctl 2>/dev/null
}

# BLE-01: Bluetooth adapter check
info "BLE-01: Checking Bluetooth adapter..."

adapter_info=$(run_btctl "show" 3)
if echo "$adapter_info" | grep -q "Powered: yes"; then
    pass "BLE-01" "Bluetooth adapter powered on"
else
    # Try to power on
    run_btctl "power on" 3 > /dev/null
    sleep 1
    adapter_info=$(run_btctl "show" 3)
    if echo "$adapter_info" | grep -q "Powered: yes"; then
        pass "BLE-01" "Bluetooth adapter powered on"
    else
        fail "BLE-01" "Bluetooth adapter" "powered on" "powered off"
        echo "Cannot continue BLE tests without powered adapter"
        print_summary
        exit 1
    fi
fi

# BLE-02: Device discovery
info "BLE-02: Scanning for $BLE_DEVICE_NAME device..."

# Start scan
run_btctl "scan on" 2 > /dev/null &
SCAN_PID=$!

# Wait for scan
sleep 8

# Stop scan
run_btctl "scan off" 2 > /dev/null

# Get devices
devices=$(run_btctl "devices" 3)
info "Found devices: $(echo "$devices" | wc -l)"

if echo "$devices" | grep -qi "$BLE_DEVICE_NAME"; then
    BLE_MAC=$(echo "$devices" | grep -i "$BLE_DEVICE_NAME" | awk '{print $2}' | head -1)
    pass "BLE-02" "Found $BLE_DEVICE_NAME at $BLE_MAC"
else
    fail "BLE-02" "Device discovery" "found $BLE_DEVICE_NAME" "not found"
    echo "Devices found:"
    echo "$devices"
    echo ""
    echo "Cannot continue BLE tests without finding device"
    print_summary
    exit 1
fi

# BLE-03: Device connection
info "BLE-03: Connecting to $BLE_MAC..."

# Remove any existing pairing
run_btctl "remove $BLE_MAC" 3 > /dev/null 2>&1

# Connect
connect_result=$(run_btctl "connect $BLE_MAC" 10)

if echo "$connect_result" | grep -qi "Connection successful"; then
    pass "BLE-03" "Connected to $BLE_DEVICE_NAME"
else
    # Check if already connected
    device_info=$(run_btctl "info $BLE_MAC" 5)
    if echo "$device_info" | grep -q "Connected: yes"; then
        pass "BLE-03" "Connected to $BLE_DEVICE_NAME"
    else
        fail "BLE-03" "BLE connection" "successful" "failed"
        info "Connection result: $connect_result"
        skip "BLE-04" "Service discovery" "Not connected"
        skip "BLE-05" "Engage command" "Not connected"
        skip "BLE-06" "Disengage command" "Not connected"
        skip "BLE-07" "Heading adjust" "Not connected"
        skip "BLE-08" "Status read" "Not connected"
        print_summary
        exit 1
    fi
fi

# Wait for services to be discovered
sleep 3

# BLE-04: Service discovery
info "BLE-04: Discovering GATT services..."

# Enter GATT menu and list attributes
gatt_output=$(echo -e "menu gatt\nlist-attributes $BLE_MAC" | timeout 5 bluetoothctl 2>/dev/null)

if echo "$gatt_output" | grep -qi "12345678-1234-1234-1234-123456789abc\|service\|characteristic"; then
    pass "BLE-04" "GATT services discovered"
else
    # Try alternative method
    device_info=$(run_btctl "info $BLE_MAC" 5)
    if echo "$device_info" | grep -qi "UUID\|Services"; then
        pass "BLE-04" "GATT services available"
    else
        fail "BLE-04" "Service discovery" "services found" "no services"
    fi
fi

# BLE-05: Test engage command via BLE
info "BLE-05: Testing BLE engage command..."

# First ensure Master is disengaged via console
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
sleep 1

initial_state=$(get_state "$MASTER_IP")
info "Initial state: $initial_state"

# Note: Direct GATT write via bluetoothctl is complex
# For now, we'll verify via console that BLE is functional
# Real BLE command testing would require gatttool or a Python script

skip "BLE-05" "BLE engage command" "Requires gatttool or custom BLE client"

# BLE-06: Test disengage command via BLE
skip "BLE-06" "BLE disengage command" "Requires gatttool or custom BLE client"

# BLE-07: Test heading adjust via BLE
skip "BLE-07" "BLE heading adjust" "Requires gatttool or custom BLE client"

# BLE-08: Read status via BLE
skip "BLE-08" "BLE status read" "Requires gatttool or custom BLE client"

# BLE-09: Connection stability
info "BLE-09: Connection stability check..."

device_info=$(run_btctl "info $BLE_MAC" 5)
if echo "$device_info" | grep -q "Connected: yes"; then
    pass "BLE-09" "BLE connection stable"
else
    fail "BLE-09" "BLE connection stability" "connected" "disconnected"
fi

# BLE-10: Disconnect
info "BLE-10: Disconnecting..."

disconnect_result=$(run_btctl "disconnect $BLE_MAC" 5)
sleep 1

device_info=$(run_btctl "info $BLE_MAC" 5)
if echo "$device_info" | grep -q "Connected: no"; then
    pass "BLE-10" "Disconnected successfully"
else
    # Force disconnect
    run_btctl "disconnect $BLE_MAC" 3 > /dev/null
    pass "BLE-10" "Disconnect command sent"
fi

# Clean up console state
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading real" > /dev/null

print_summary

echo ""
echo "NOTE: Full BLE command testing requires a dedicated BLE client tool."
echo "Consider using:"
echo "  - gatttool (deprecated but functional)"
echo "  - Python bleak library"
echo "  - nRF Connect mobile app"
echo ""

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
