#!/bin/bash
# TestAPEN Test Common Library
# Source this file in all test scripts

# Node IP Addresses
MASTER_IP="192.168.1.118"
RUDDER_IP="192.168.1.157"
UI_IP="192.168.1.186"
CONSOLE_PORT="2323"

# Test result counters
PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Results file
RESULTS_DIR="/home/cas/TestAPEN/tests/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RESULTS_FILE="${RESULTS_DIR}/test_results_${TIMESTAMP}.log"

# Initialize results file
init_results() {
    mkdir -p "$RESULTS_DIR"
    echo "TestAPEN Test Results - $(date)" > "$RESULTS_FILE"
    echo "========================================" >> "$RESULTS_FILE"
    echo "" >> "$RESULTS_FILE"
}

# Send command to node and capture output
# Usage: send_cmd <IP> <command> [timeout_seconds]
send_cmd() {
    local ip="$1"
    local cmd="$2"
    local timeout="${3:-2}"

    (echo "$cmd"; sleep "$timeout") | nc -q 2 "$ip" "$CONSOLE_PORT" 2>/dev/null
}

# Send command and get specific field from output
# Usage: get_field <IP> <command> <grep_pattern>
get_field() {
    local ip="$1"
    local cmd="$2"
    local pattern="$3"

    send_cmd "$ip" "$cmd" | grep -i "$pattern" | head -1
}

# Check if node is reachable
# Usage: check_node <IP> <name>
check_node() {
    local ip="$1"
    local name="$2"

    if nc -z -w 2 "$ip" "$CONSOLE_PORT" 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

# Log test result - PASS
# Usage: pass <test_id> <description>
pass() {
    local test_id="$1"
    local desc="$2"

    echo -e "${GREEN}[PASS]${NC} $test_id: $desc"
    echo "[PASS] $test_id: $desc" >> "$RESULTS_FILE"
    ((PASS_COUNT++))
}

# Log test result - FAIL
# Usage: fail <test_id> <description> [expected] [actual]
fail() {
    local test_id="$1"
    local desc="$2"
    local expected="${3:-}"
    local actual="${4:-}"

    if [ -n "$expected" ] && [ -n "$actual" ]; then
        echo -e "${RED}[FAIL]${NC} $test_id: $desc - Expected: $expected, Got: $actual"
        echo "[FAIL] $test_id: $desc - Expected: $expected, Got: $actual" >> "$RESULTS_FILE"
    else
        echo -e "${RED}[FAIL]${NC} $test_id: $desc"
        echo "[FAIL] $test_id: $desc" >> "$RESULTS_FILE"
    fi
    ((FAIL_COUNT++))
}

# Log test result - SKIP
# Usage: skip <test_id> <description> <reason>
skip() {
    local test_id="$1"
    local desc="$2"
    local reason="$3"

    echo -e "${YELLOW}[SKIP]${NC} $test_id: $desc - $reason"
    echo "[SKIP] $test_id: $desc - $reason" >> "$RESULTS_FILE"
    ((SKIP_COUNT++))
}

# Log info message
# Usage: info <message>
info() {
    echo -e "${BLUE}[INFO]${NC} $1"
    echo "[INFO] $1" >> "$RESULTS_FILE"
}

# Print test summary
print_summary() {
    echo ""
    echo "========================================"
    echo "Test Summary"
    echo "========================================"
    echo -e "Passed:  ${GREEN}$PASS_COUNT${NC}"
    echo -e "Failed:  ${RED}$FAIL_COUNT${NC}"
    echo -e "Skipped: ${YELLOW}$SKIP_COUNT${NC}"
    echo "Total:   $((PASS_COUNT + FAIL_COUNT + SKIP_COUNT))"
    echo ""
    echo "Results saved to: $RESULTS_FILE"

    echo "" >> "$RESULTS_FILE"
    echo "========================================" >> "$RESULTS_FILE"
    echo "Summary: PASS=$PASS_COUNT FAIL=$FAIL_COUNT SKIP=$SKIP_COUNT" >> "$RESULTS_FILE"
}

# Wait for node to come online after reboot
# Usage: wait_for_node <IP> <name> [timeout_seconds]
wait_for_node() {
    local ip="$1"
    local name="$2"
    local timeout="${3:-30}"
    local elapsed=0

    info "Waiting for $name ($ip) to come online..."
    while [ $elapsed -lt $timeout ]; do
        if check_node "$ip" "$name"; then
            info "$name online after ${elapsed}s"
            return 0
        fi
        sleep 1
        ((elapsed++))
    done

    return 1
}

# Get current state from node
# Usage: get_state <IP>
get_state() {
    local ip="$1"
    send_cmd "$ip" "state" | grep -i "state" | grep -oE "(BOOT|IDLE|ENGAGED|CALIBRATION|FAULTED)" | head -1
}

# Get fault code from node
# Usage: get_fault_code <IP>
get_fault_code() {
    local ip="$1"
    send_cmd "$ip" "state" | grep -i "fault" | grep -oE "0x[0-9A-Fa-f]+" | head -1
}

# Assert state equals expected
# Usage: assert_state <IP> <expected_state> <test_id> <description>
assert_state() {
    local ip="$1"
    local expected="$2"
    local test_id="$3"
    local desc="$4"

    local actual=$(get_state "$ip")

    if [ "$actual" = "$expected" ]; then
        pass "$test_id" "$desc"
        return 0
    else
        fail "$test_id" "$desc" "$expected" "$actual"
        return 1
    fi
}

# Assert output contains string
# Usage: assert_contains <output> <expected_substring> <test_id> <description>
assert_contains() {
    local output="$1"
    local expected="$2"
    local test_id="$3"
    local desc="$4"

    if echo "$output" | grep -q "$expected"; then
        pass "$test_id" "$desc"
        return 0
    else
        fail "$test_id" "$desc" "contains '$expected'" "not found"
        return 1
    fi
}

# Get ESP-NOW RX count
# Usage: get_rx_count <IP>
get_rx_count() {
    local ip="$1"
    send_cmd "$ip" "espnow" | grep "RX count" | grep -oE "[0-9]+" | head -1
}

# Get ESP-NOW TX failed count
# Usage: get_tx_failed <IP>
get_tx_failed() {
    local ip="$1"
    send_cmd "$ip" "espnow" | grep "TX failed" | grep -oE "[0-9]+" | head -1
}

# Get rudder angle
# Usage: get_rudder_angle <IP>
get_rudder_angle() {
    local ip="$1"
    send_cmd "$ip" "rudder" | grep -i "actual" | grep -oE "[-]?[0-9]+\.[0-9]+" | head -1
}

# Get heading
# Usage: get_heading <IP>
get_heading() {
    local ip="$1"
    send_cmd "$ip" "heading" | grep -i "current" | grep -oE "[0-9]+\.[0-9]+" | head -1
}

# Get target heading
# Usage: get_target <IP>
get_target() {
    local ip="$1"
    send_cmd "$ip" "heading" | grep -i "target" | grep -oE "[0-9]+\.[0-9]+" | head -1
}

# Take camera snapshot (placeholder - implement based on camera setup)
# Usage: take_snapshot <output_file> <description>
take_snapshot() {
    local output_file="$1"
    local description="$2"

    info "Taking snapshot: $description"
    # This is a placeholder - implement based on your camera setup
    # Example using fswebcam:
    # fswebcam -r 1280x720 --no-banner "$output_file"

    # For now, just create a placeholder
    echo "Snapshot placeholder: $description" > "${output_file}.txt"
}

# Export functions
export -f send_cmd get_field check_node pass fail skip info
export -f print_summary wait_for_node get_state get_fault_code
export -f assert_state assert_contains get_rx_count get_tx_failed
export -f get_rudder_angle get_heading get_target take_snapshot
export MASTER_IP RUDDER_IP UI_IP CONSOLE_PORT
export RESULTS_DIR RESULTS_FILE
