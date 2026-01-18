#!/bin/bash
# TestAPEN Button Tests
# Interactive tests for UI node buttons

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Button Tests (Interactive)"
echo "========================================"
echo ""
echo "This test requires physical access to the UI node buttons."
echo ""

init_results

# BTN-01: Button state query
info "BTN-01: Button state query..."

button_status=$(send_cmd "$UI_IP" "buttons")
if echo "$button_status" | grep -qiE "button|gpio|dec|inc|engage|mode"; then
    pass "BTN-01" "Button status command works"
    echo "$button_status"
else
    fail "BTN-01" "Button status query"
fi

echo ""
echo "=== Interactive Button Tests ==="
echo "Follow the prompts to test each button."
echo ""

# Helper function for interactive button test
test_button() {
    local button_name="$1"
    local expected_action="$2"
    local test_id="$3"
    local check_field="$4"
    local check_value="$5"

    echo ""
    echo "[$test_id] Testing: $button_name"
    echo "Expected action: $expected_action"
    echo ""

    # Get initial value
    local initial_value=""
    case "$check_field" in
        "target")
            initial_value=$(get_target "$MASTER_IP")
            ;;
        "state")
            initial_value=$(get_state "$MASTER_IP")
            ;;
    esac

    echo "Current $check_field: $initial_value"
    echo ""
    read -p "Press the $button_name button on UI node, then press ENTER here... " dummy

    sleep 1

    # Get new value
    local new_value=""
    case "$check_field" in
        "target")
            new_value=$(get_target "$MASTER_IP")
            ;;
        "state")
            new_value=$(get_state "$MASTER_IP")
            ;;
    esac

    echo "New $check_field: $new_value"

    if [ "$new_value" != "$initial_value" ]; then
        pass "$test_id" "$button_name changes $check_field"
    else
        fail "$test_id" "$button_name" "$check_field changed" "no change"
    fi
}

# Ensure system is in known state
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
sleep 1

# BTN-02: DEC-10 button
echo ""
echo "=== BTN-02: DEC-10 Button (-10 degrees) ==="
test_button "DEC-10" "Decrease heading by 10 degrees" "BTN-02" "target"

# BTN-03: DEC-1 button
echo ""
echo "=== BTN-03: DEC-1 Button (-1 degree) ==="
test_button "DEC-1" "Decrease heading by 1 degree" "BTN-03" "target"

# BTN-04: INC+1 button
echo ""
echo "=== BTN-04: INC+1 Button (+1 degree) ==="
test_button "INC+1" "Increase heading by 1 degree" "BTN-04" "target"

# BTN-05: INC+10 button
echo ""
echo "=== BTN-05: INC+10 Button (+10 degrees) ==="
test_button "INC+10" "Increase heading by 10 degrees" "BTN-05" "target"

# BTN-06: ENGAGE button
echo ""
echo "=== BTN-06: ENGAGE Button ==="
current_state=$(get_state "$MASTER_IP")
echo "Current state: $current_state"
expected_state="ENGAGED"
if [ "$current_state" = "ENGAGED" ]; then
    expected_state="IDLE"
fi
echo "Expected after press: $expected_state"
echo ""
read -p "Press the ENGAGE button on UI node, then press ENTER here... " dummy
sleep 1

new_state=$(get_state "$MASTER_IP")
echo "New state: $new_state"

if [ "$new_state" = "$expected_state" ]; then
    pass "BTN-06" "ENGAGE button toggles state"
else
    fail "BTN-06" "ENGAGE button" "$expected_state" "$new_state"
fi

# BTN-07: MODE button (page cycle)
echo ""
echo "=== BTN-07: MODE Button (Page Cycle) ==="
echo "This test requires visual verification of the e-Paper display."
echo "Current page should change when MODE is pressed."
echo ""
read -p "Press the MODE button on UI node and watch the display. Press ENTER when done... " dummy

read -p "Did the display page change? (y/n): " page_changed
if [ "$page_changed" = "y" ] || [ "$page_changed" = "Y" ]; then
    pass "BTN-07" "MODE button cycles pages"
else
    fail "BTN-07" "MODE button page cycle"
fi

# BTN-08: Long press repeat
echo ""
echo "=== BTN-08: Long Press Repeat Test ==="
echo "Hold the INC+10 button for 3 seconds to test auto-repeat."
echo ""

# Get initial target
initial_target=$(get_target "$MASTER_IP")
echo "Initial target heading: $initial_target"
echo ""
read -p "HOLD the INC+10 button for 3 seconds, then release and press ENTER... " dummy

sleep 0.5
final_target=$(get_target "$MASTER_IP")
echo "Final target heading: $final_target"

if [ -n "$initial_target" ] && [ -n "$final_target" ]; then
    diff=$(echo "$final_target - $initial_target" | bc -l 2>/dev/null | sed 's/-//')
    if [ -n "$diff" ]; then
        large_change=$(echo "$diff > 20" | bc -l 2>/dev/null)
        if [ "$large_change" = "1" ]; then
            pass "BTN-08" "Long press auto-repeat works (changed by $diff deg)"
        else
            fail "BTN-08" "Long press repeat" ">20 deg change" "$diff deg"
        fi
    else
        skip "BTN-08" "Long press repeat" "Could not calculate difference"
    fi
else
    skip "BTN-08" "Long press repeat" "Could not get target values"
fi

# Clean up
send_cmd "$MASTER_IP" "disengage" > /dev/null
send_cmd "$MASTER_IP" "heading real" > /dev/null

print_summary

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
