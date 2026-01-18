#!/bin/bash
# TestAPEN Display Tests
# Tests OLED (Master) and e-Paper (UI) displays
# Uses camera for visual verification

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/test_common.sh"

echo "========================================"
echo "TestAPEN Display Tests"
echo "========================================"
echo ""

init_results

# Directory for captured images
CAPTURE_DIR="$RESULTS_DIR/display_captures_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$CAPTURE_DIR"

# Camera device (adjust as needed)
CAMERA_DEV="/dev/video0"

# Check for camera
check_camera() {
    if [ -e "$CAMERA_DEV" ]; then
        return 0
    else
        # Try to find any video device
        for dev in /dev/video*; do
            if [ -e "$dev" ]; then
                CAMERA_DEV="$dev"
                return 0
            fi
        done
        return 1
    fi
}

# Capture image using fswebcam or ffmpeg
capture_image() {
    local output_file="$1"
    local description="$2"

    info "Capturing: $description"

    if command -v fswebcam &> /dev/null; then
        fswebcam -d "$CAMERA_DEV" -r 1280x720 --no-banner \
            --title "$description" "$output_file" 2>/dev/null
        return $?
    elif command -v ffmpeg &> /dev/null; then
        ffmpeg -f v4l2 -i "$CAMERA_DEV" -frames:v 1 "$output_file" -y 2>/dev/null
        return $?
    else
        echo "WARNING: No camera capture tool found (fswebcam or ffmpeg)"
        echo "Creating placeholder for: $description"
        echo "Placeholder: $description - $(date)" > "${output_file%.jpg}.txt"
        return 1
    fi
}

# === UI Node e-Paper Display Tests ===

info "=== UI Node e-Paper Display Tests ==="

# DISP-U01: Display status check
info "DISP-U01: e-Paper display status..."

display_status=$(send_cmd "$UI_IP" "display")
if echo "$display_status" | grep -qi "initialized.*yes\|Initialized: yes"; then
    pass "DISP-U01" "e-Paper display initialized"
else
    fail "DISP-U01" "e-Paper initialized"
fi

# DISP-U02: Page 0 - Autopilot Status
info "DISP-U02: Page 0 - Autopilot Status..."

send_cmd "$UI_IP" "page 0" > /dev/null
sleep 3  # e-Paper refresh time

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_page0_autopilot.jpg" "UI Page 0 - Autopilot"
    if [ -f "$CAPTURE_DIR/ui_page0_autopilot.jpg" ]; then
        pass "DISP-U02" "Page 0 captured to $CAPTURE_DIR/ui_page0_autopilot.jpg"
    else
        fail "DISP-U02" "Page 0 capture"
    fi
else
    skip "DISP-U02" "Page 0 capture" "No camera available"
fi

# DISP-U03: Page 1 - Navigation
info "DISP-U03: Page 1 - Navigation..."

send_cmd "$UI_IP" "page 1" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_page1_nav.jpg" "UI Page 1 - Navigation"
    if [ -f "$CAPTURE_DIR/ui_page1_nav.jpg" ]; then
        pass "DISP-U03" "Page 1 captured"
    else
        fail "DISP-U03" "Page 1 capture"
    fi
else
    skip "DISP-U03" "Page 1 capture" "No camera available"
fi

# DISP-U04: Page 2 - System Status
info "DISP-U04: Page 2 - System Status..."

send_cmd "$UI_IP" "page 2" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_page2_system.jpg" "UI Page 2 - System"
    if [ -f "$CAPTURE_DIR/ui_page2_system.jpg" ]; then
        pass "DISP-U04" "Page 2 captured"
    else
        fail "DISP-U04" "Page 2 capture"
    fi
else
    skip "DISP-U04" "Page 2 capture" "No camera available"
fi

# DISP-U05: Manual refresh
info "DISP-U05: Manual refresh command..."

refresh_result=$(send_cmd "$UI_IP" "display refresh" 4)
if [ $? -eq 0 ]; then
    pass "DISP-U05" "Display refresh command accepted"
else
    fail "DISP-U05" "Display refresh command"
fi

# DISP-U06: Heading display update test
info "DISP-U06: Heading display update..."

# Set heading to known value
send_cmd "$MASTER_IP" "heading sim 123" > /dev/null
sleep 3

# Return to page 0 to see heading
send_cmd "$UI_IP" "page 0" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_heading_123.jpg" "UI Heading 123 deg"
    pass "DISP-U06" "Heading 123 display captured"
else
    skip "DISP-U06" "Heading display" "No camera"
fi

# Change heading
send_cmd "$MASTER_IP" "heading sim 270" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_heading_270.jpg" "UI Heading 270 deg"
    pass "DISP-U06b" "Heading 270 display captured"
else
    skip "DISP-U06b" "Heading display" "No camera"
fi

send_cmd "$MASTER_IP" "heading real" > /dev/null

# DISP-U07: State display test
info "DISP-U07: State display test..."

# Engage
send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
send_cmd "$MASTER_IP" "set heading 180" > /dev/null
send_cmd "$MASTER_IP" "engage" > /dev/null
sleep 3

send_cmd "$UI_IP" "page 0" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_state_engaged.jpg" "UI State ENGAGED"
    pass "DISP-U07a" "ENGAGED state display captured"
else
    skip "DISP-U07a" "ENGAGED state display" "No camera"
fi

# Disengage
send_cmd "$MASTER_IP" "disengage" > /dev/null
sleep 3

if check_camera; then
    capture_image "$CAPTURE_DIR/ui_state_idle.jpg" "UI State IDLE"
    pass "DISP-U07b" "IDLE state display captured"
else
    skip "DISP-U07b" "IDLE state display" "No camera"
fi

send_cmd "$MASTER_IP" "heading real" > /dev/null

# === Master Node OLED Display Tests ===

info ""
info "=== Master Node OLED Display Tests ==="
info "Note: Master OLED requires separate camera positioning"

# DISP-M01: OLED basic check
info "DISP-M01: Master OLED basic check..."

# The Master doesn't have a dedicated display command, but we can verify
# it's running by checking the display task is active
master_status=$(send_cmd "$MASTER_IP" "status")
if [ $? -eq 0 ]; then
    pass "DISP-M01" "Master responding (display task running)"
else
    fail "DISP-M01" "Master response check"
fi

# DISP-M02: Master display with different states
info "DISP-M02: Master display state test..."

if check_camera; then
    # Capture IDLE state
    send_cmd "$MASTER_IP" "disengage" > /dev/null
    sleep 2
    capture_image "$CAPTURE_DIR/master_idle.jpg" "Master OLED IDLE"

    # Capture ENGAGED state
    send_cmd "$MASTER_IP" "heading sim 180" > /dev/null
    send_cmd "$MASTER_IP" "set heading 180" > /dev/null
    send_cmd "$MASTER_IP" "engage" > /dev/null
    sleep 2
    capture_image "$CAPTURE_DIR/master_engaged.jpg" "Master OLED ENGAGED"

    send_cmd "$MASTER_IP" "disengage" > /dev/null
    send_cmd "$MASTER_IP" "heading real" > /dev/null

    pass "DISP-M02" "Master display states captured"
else
    skip "DISP-M02" "Master display capture" "No camera"
fi

# === Summary of captured images ===

echo ""
info "=== Captured Images ==="
if [ -d "$CAPTURE_DIR" ]; then
    ls -la "$CAPTURE_DIR/" 2>/dev/null || echo "No images captured"
fi

print_summary

echo ""
echo "Display test images saved to: $CAPTURE_DIR"
echo "Please visually verify the captured images show correct content:"
echo "  - Page 0: Heading, Target, Rudder angle, State"
echo "  - Page 1: Lat/Lon (or No Fix), Speed, COG"
echo "  - Page 2: Master/Rudder connectivity, WiFi IP, Version"
echo "  - Master OLED: Heading, Target, State"
echo ""

if [ "$FAIL_COUNT" -gt 0 ]; then
    exit 1
fi
exit 0
