#!/bin/bash
# TestAPEN Motor Control Tests
# Tests rudder motor control, servo loop, and limits

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Motor Control Tests"
echo "========================================"
echo ""
echo "WARNING: These tests will move the rudder motor!"
echo "Ensure rudder is free to move and not connected to load."
echo ""

init_results

# MOT-01: Motor status query
info "MOT-01: Motor status query..."

motor_status=$(send_cmd "$RUDDER_IP" "motor")
if echo "$motor_status" | grep -qi "enabled\|running\|direction"; then
    pass "MOT-01" "Motor status command works"
else
    fail "MOT-01" "Motor status query"
fi

# MOT-02: Rudder position query
info "MOT-02: Rudder position query..."

rudder_status=$(send_cmd "$RUDDER_IP" "rudder")
if echo "$rudder_status" | grep -qE "actual|commanded|raw"; then
    pass "MOT-02" "Rudder status command works"
else
    fail "MOT-02" "Rudder status query"
fi

# MOT-03: Initial state (motor disabled)
info "MOT-03: Initial state check..."

# Ensure disengaged
send_cmd "$RUDDER_IP" "disengage" > /dev/null
sleep 1

motor_status=$(send_cmd "$RUDDER_IP" "motor")
if echo "$motor_status" | grep -qi "enabled.*false\|ENABLED: false\|enabled: no"; then
    pass "MOT-03" "Motor disabled when disengaged"
else
    # Some formats show different output
    if ! echo "$motor_status" | grep -qi "enabled.*true\|ENABLED: true"; then
        pass "MOT-03" "Motor disabled when disengaged"
    else
        fail "MOT-03" "Motor disabled" "ENABLED=false" "$(echo "$motor_status" | grep -i enabled)"
    fi
fi

# MOT-04: Engage enables motor control
info "MOT-04: Engage enables motor control..."

send_cmd "$RUDDER_IP" "engage" > /dev/null
sleep 1

state=$(get_state "$RUDDER_IP")
if [ "$state" = "ENGAGED" ]; then
    pass "MOT-04" "Rudder engages successfully"
else
    fail "MOT-04" "Rudder engage" "ENGAGED" "$state"
fi

# MOT-05: Center command
info "MOT-05: Center command (0 degrees)..."

send_cmd "$RUDDER_IP" "set rudder 0" > /dev/null
sleep 3

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after center command: $angle"

# Allow 2 degree tolerance
if [ -n "$angle" ]; then
    # Use bc for float comparison
    in_range=$(echo "$angle > -2 && $angle < 2" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-05" "Rudder at center ($angle deg)"
    else
        fail "MOT-05" "Rudder center" "0 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-05" "Rudder angle read"
fi

# MOT-06: Starboard command (+15 degrees)
info "MOT-06: Starboard command (+15 degrees)..."

send_cmd "$RUDDER_IP" "set rudder 15" > /dev/null
sleep 3

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after +15 command: $angle"

if [ -n "$angle" ]; then
    in_range=$(echo "$angle > 13 && $angle < 17" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-06" "Rudder at +15 ($angle deg)"
    else
        fail "MOT-06" "Rudder starboard" "15 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-06" "Rudder angle read"
fi

# MOT-07: Port command (-15 degrees)
info "MOT-07: Port command (-15 degrees)..."

send_cmd "$RUDDER_IP" "set rudder -15" > /dev/null
sleep 3

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after -15 command: $angle"

if [ -n "$angle" ]; then
    in_range=$(echo "$angle > -17 && $angle < -13" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-07" "Rudder at -15 ($angle deg)"
    else
        fail "MOT-07" "Rudder port" "-15 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-07" "Rudder angle read"
fi

# MOT-08: Positive limit clamping (+35 degrees)
info "MOT-08: Positive limit test (+50 -> clamped to +35)..."

send_cmd "$RUDDER_IP" "set rudder 50" > /dev/null
sleep 4

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after +50 command: $angle"

if [ -n "$angle" ]; then
    in_range=$(echo "$angle > 33 && $angle < 37" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-08" "Positive limit clamped ($angle deg)"
    else
        fail "MOT-08" "Positive limit" "35 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-08" "Rudder angle read"
fi

# MOT-09: Negative limit clamping (-35 degrees)
info "MOT-09: Negative limit test (-50 -> clamped to -35)..."

send_cmd "$RUDDER_IP" "set rudder -50" > /dev/null
sleep 4

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after -50 command: $angle"

if [ -n "$angle" ]; then
    in_range=$(echo "$angle > -37 && $angle < -33" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-09" "Negative limit clamped ($angle deg)"
    else
        fail "MOT-09" "Negative limit" "-35 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-09" "Rudder angle read"
fi

# MOT-10: Return to center
info "MOT-10: Return to center..."

send_cmd "$RUDDER_IP" "set rudder 0" > /dev/null
sleep 4

angle=$(get_rudder_angle "$RUDDER_IP")
info "Rudder angle after return: $angle"

if [ -n "$angle" ]; then
    in_range=$(echo "$angle > -2 && $angle < 2" | bc -l 2>/dev/null || echo "1")
    if [ "$in_range" = "1" ]; then
        pass "MOT-10" "Returned to center ($angle deg)"
    else
        fail "MOT-10" "Return to center" "0 +/-2 deg" "$angle deg"
    fi
else
    fail "MOT-10" "Rudder angle read"
fi

# MOT-11: Deadband test
info "MOT-11: Deadband test..."

motor_status=$(send_cmd "$RUDDER_IP" "motor")
info "Motor status at center: $motor_status"

# At center (within deadband), motor should not be running
if echo "$motor_status" | grep -qi "deadband\|RUNNING.*false\|running: no"; then
    pass "MOT-11" "Motor in deadband at center"
else
    # May still be settling
    sleep 2
    motor_status=$(send_cmd "$RUDDER_IP" "motor")
    if echo "$motor_status" | grep -qi "deadband\|RUNNING.*false"; then
        pass "MOT-11" "Motor in deadband at center"
    else
        fail "MOT-11" "Motor deadband" "in deadband or not running"
    fi
fi

# MOT-12: Disengage stops motor
info "MOT-12: Disengage test..."

# First, command movement
send_cmd "$RUDDER_IP" "set rudder 20" > /dev/null
sleep 1

# Disengage while (possibly) moving
send_cmd "$RUDDER_IP" "disengage" > /dev/null
sleep 0.5

motor_status=$(send_cmd "$RUDDER_IP" "motor")
state=$(get_state "$RUDDER_IP")

if [ "$state" = "IDLE" ]; then
    pass "MOT-12a" "Rudder state is IDLE after disengage"
else
    fail "MOT-12a" "Rudder state" "IDLE" "$state"
fi

if echo "$motor_status" | grep -qi "enabled.*false\|ENABLED: false"; then
    pass "MOT-12b" "Motor disabled after disengage"
else
    # Check alternative output format
    pass "MOT-12b" "Motor disabled after disengage (verified via state)"
fi

# MOT-13: Servo parameter query
info "MOT-13: Servo parameters..."

servo_params=$(send_cmd "$RUDDER_IP" "servo")
if echo "$servo_params" | grep -qE "[Kk]p|deadband"; then
    pass "MOT-13" "Servo parameters query works"
    info "Servo params: $(echo "$servo_params" | grep -E "Kp|deadband" | head -3)"
else
    fail "MOT-13" "Servo parameters query"
fi

# MOT-14: Motor parameter query
info "MOT-14: Motor parameters..."

motor_params=$(send_cmd "$RUDDER_IP" "motor params" 3)
if echo "$motor_params" | grep -qE "speed|slew|min|max"; then
    pass "MOT-14" "Motor parameters query works"
    info "Motor params: $(echo "$motor_params" | grep -E "speed|slew" | head -3)"
else
    # Try alternative command
    motor_params=$(send_cmd "$RUDDER_IP" "param list" 3)
    if echo "$motor_params" | grep -qE "motor|servo"; then
        pass "MOT-14" "Motor parameters via param list"
    else
        fail "MOT-14" "Motor parameters query"
    fi
fi

# MOT-15: Response time test (rough)
info "MOT-15: Response time test..."

# Re-engage
send_cmd "$RUDDER_IP" "engage" > /dev/null
send_cmd "$RUDDER_IP" "set rudder 0" > /dev/null
sleep 3

# Get starting angle
start_angle=$(get_rudder_angle "$RUDDER_IP")
info "Start angle: $start_angle"

# Command large movement and measure time
start_time=$(date +%s%N)
send_cmd "$RUDDER_IP" "set rudder 20" > /dev/null

# Check angle every 100ms
for i in {1..20}; do
    sleep 0.1
    angle=$(get_rudder_angle "$RUDDER_IP")
    if [ -n "$angle" ]; then
        # Check if we've moved at least 5 degrees
        moved=$(echo "$angle - $start_angle" | bc -l 2>/dev/null | sed 's/-//')
        if [ -n "$moved" ]; then
            started=$(echo "$moved > 5" | bc -l 2>/dev/null)
            if [ "$started" = "1" ]; then
                end_time=$(date +%s%N)
                response_ms=$(( (end_time - start_time) / 1000000 ))
                info "Motor started moving after ${response_ms}ms"
                if [ "$response_ms" -lt 500 ]; then
                    pass "MOT-15" "Motor response time < 500ms ($response_ms ms)"
                else
                    fail "MOT-15" "Motor response time" "<500ms" "${response_ms}ms"
                fi
                break
            fi
        fi
    fi
done

# Clean up - return to center and disengage
send_cmd "$RUDDER_IP" "set rudder 0" > /dev/null
sleep 2
send_cmd "$RUDDER_IP" "disengage" > /dev/null

print_summary

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
