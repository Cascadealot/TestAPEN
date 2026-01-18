#!/bin/bash
# TestAPEN Master Test Runner
# Runs all test suites and generates comprehensive report

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="/home/cas/TestAPEN/tests/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
MASTER_RESULTS="$RESULTS_DIR/full_test_run_$TIMESTAMP"

echo "========================================"
echo "TestAPEN Complete Test Suite"
echo "========================================"
echo "Timestamp: $(date)"
echo "Results directory: $MASTER_RESULTS"
echo ""

mkdir -p "$MASTER_RESULTS"

# Source common functions for node checking
source "$SCRIPT_DIR/test_common.sh"

# Initialize master results file
SUMMARY_FILE="$MASTER_RESULTS/summary.txt"
echo "TestAPEN Test Run Summary - $(date)" > "$SUMMARY_FILE"
echo "========================================" >> "$SUMMARY_FILE"
echo "" >> "$SUMMARY_FILE"

# Function to run a test suite
run_suite() {
    local suite_name="$1"
    local script="$2"
    local suite_log="$MASTER_RESULTS/${suite_name}.log"

    echo ""
    echo "========================================"
    echo "Running: $suite_name"
    echo "========================================"

    if [ -x "$script" ]; then
        "$script" 2>&1 | tee "$suite_log"
        local exit_code=${PIPESTATUS[0]}

        if [ $exit_code -eq 0 ]; then
            echo "[$suite_name] PASSED" >> "$SUMMARY_FILE"
        else
            echo "[$suite_name] FAILED (exit code: $exit_code)" >> "$SUMMARY_FILE"
        fi

        return $exit_code
    else
        echo "ERROR: Script not found or not executable: $script"
        echo "[$suite_name] ERROR - Script not found" >> "$SUMMARY_FILE"
        return 1
    fi
}

# Track overall results
SUITES_PASSED=0
SUITES_FAILED=0
SUITES_SKIPPED=0

# Pre-flight checks
echo ""
echo "=== Pre-flight Checks ==="
echo ""

# Check if nodes are reachable
MASTER_IP="192.168.1.118"
RUDDER_IP="192.168.1.157"
UI_IP="192.168.1.186"

nodes_ok=true

echo -n "Checking Master ($MASTER_IP)... "
if nc -z -w 2 "$MASTER_IP" 2323 2>/dev/null; then
    echo "OK"
else
    echo "UNREACHABLE"
    nodes_ok=false
fi

echo -n "Checking Rudder ($RUDDER_IP)... "
if nc -z -w 2 "$RUDDER_IP" 2323 2>/dev/null; then
    echo "OK"
else
    echo "UNREACHABLE"
    nodes_ok=false
fi

echo -n "Checking UI ($UI_IP)... "
if nc -z -w 2 "$UI_IP" 2323 2>/dev/null; then
    echo "OK"
else
    echo "UNREACHABLE"
    nodes_ok=false
fi

if [ "$nodes_ok" = false ]; then
    echo ""
    echo "ERROR: One or more nodes are unreachable."
    echo "Please ensure all nodes are powered on and connected to WiFi."
    echo ""
    echo "Aborting test run."
    exit 1
fi

echo ""
echo "All nodes reachable. Starting tests..."
echo ""

# Make all scripts executable
chmod +x "$SCRIPT_DIR"/*.sh

# Run test suites in order

# 1. Quick Validation (must pass to continue)
echo "" >> "$SUMMARY_FILE"
echo "=== Quick Validation ===" >> "$SUMMARY_FILE"
run_suite "quick_validation" "$SCRIPT_DIR/quick_validation.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
    echo ""
    echo "WARNING: Quick validation failed. Continuing with remaining tests..."
fi

# 2. Communication Tests
echo "" >> "$SUMMARY_FILE"
echo "=== Communication Tests ===" >> "$SUMMARY_FILE"
run_suite "comm_tests" "$SCRIPT_DIR/comm_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# 3. Motor Control Tests
echo "" >> "$SUMMARY_FILE"
echo "=== Motor Control Tests ===" >> "$SUMMARY_FILE"
run_suite "motor_tests" "$SCRIPT_DIR/motor_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# 4. Error Handling Tests
echo "" >> "$SUMMARY_FILE"
echo "=== Error Handling Tests ===" >> "$SUMMARY_FILE"
run_suite "error_tests" "$SCRIPT_DIR/error_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# 5. Display Tests
echo "" >> "$SUMMARY_FILE"
echo "=== Display Tests ===" >> "$SUMMARY_FILE"
run_suite "display_tests" "$SCRIPT_DIR/display_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# 6. BLE Tests
echo "" >> "$SUMMARY_FILE"
echo "=== BLE Tests ===" >> "$SUMMARY_FILE"
run_suite "ble_tests" "$SCRIPT_DIR/ble_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# 7. FSD Acceptance Criteria Tests
echo "" >> "$SUMMARY_FILE"
echo "=== Acceptance Criteria Tests ===" >> "$SUMMARY_FILE"
run_suite "acceptance_tests" "$SCRIPT_DIR/acceptance_tests.sh"
if [ $? -eq 0 ]; then
    ((SUITES_PASSED++))
else
    ((SUITES_FAILED++))
fi

# Final summary
echo ""
echo "========================================"
echo "TEST RUN COMPLETE"
echo "========================================"
echo ""
echo "Test Suites Passed:  $SUITES_PASSED"
echo "Test Suites Failed:  $SUITES_FAILED"
echo "Test Suites Skipped: $SUITES_SKIPPED"
echo ""
echo "Results saved to: $MASTER_RESULTS"
echo ""

# Write final summary
echo "" >> "$SUMMARY_FILE"
echo "========================================" >> "$SUMMARY_FILE"
echo "FINAL RESULTS" >> "$SUMMARY_FILE"
echo "  Suites Passed:  $SUITES_PASSED" >> "$SUMMARY_FILE"
echo "  Suites Failed:  $SUITES_FAILED" >> "$SUMMARY_FILE"
echo "  Suites Skipped: $SUITES_SKIPPED" >> "$SUMMARY_FILE"
echo "========================================" >> "$SUMMARY_FILE"
echo "Test run completed: $(date)" >> "$SUMMARY_FILE"

# Display summary file
echo "=== Summary ==="
cat "$SUMMARY_FILE"

# Copy all individual result files to master directory
for f in "$RESULTS_DIR"/test_results_*.log; do
    if [ -f "$f" ]; then
        cp "$f" "$MASTER_RESULTS/" 2>/dev/null
    fi
done

# Exit with appropriate code
if [ $SUITES_FAILED -gt 0 ]; then
    exit 1
fi
exit 0
