#!/bin/bash
# demo.sh - Automated demonstration of ILC capabilities

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# Helper function to send command with error handling
send_cmd() {
    local response
    response=$(echo "$1" | timeout 2 ./build/ilc_client 2>&1)
    local exit_code=$?
    
    if [ $exit_code -ne 0 ]; then
        echo -e "${RED}Error: Failed to send command '$1'${NC}"
        echo "Make sure the simulator is running in another terminal!"
        exit 1
    fi
    sleep 0.5
}

# Helper function to check if simulator is reachable
check_simulator() {
    echo "Checking simulator connection..."
    if ! timeout 2 bash -c 'echo "status" | ./build/ilc_client' > /dev/null 2>&1; then
        echo -e "${RED}ERROR: Cannot connect to simulator!${NC}"
        echo ""
        echo "Please make sure the simulator is running:"
        echo "  Terminal 1: ./build/ilc_simulator"
        echo "  (or: make run-sim)"
        echo ""
        echo "Then run this demo again in Terminal 2"
        exit 1
    fi
    echo -e "${GREEN}✓ Simulator connection OK${NC}"
    echo ""
}

# Helper function to display status
show_status() {
    echo -e "${CYAN}Current Status:${NC}"
    echo "status" | timeout 2 ./build/ilc_client 2>/dev/null | grep -E "(Iteration|RMS Error|Error Level)"
}

# Check if client exists
if [[ ! -f build/ilc_client ]]; then
    echo -e "${RED}Error: Client not found. Run ./build.sh first${NC}"
    exit 1
fi

echo "========================================"
echo "  ILC Real-Time Tracker Demo"
echo "========================================"
echo ""

# Check simulator connection
check_simulator

echo "This demo will showcase:"
echo "  1. Basic circle tracking"
echo "  2. Error induction and ILC learning"
echo "  3. Different error scenarios"
echo "  4. Shape morphing"
echo "  5. Parameter tuning effects"
echo ""
read -p "Press Enter to start demo..."
echo ""

# Demo 1: Basic Operation
echo -e "${GREEN}=== Demo 1: Basic Circle Tracking ===${NC}"
echo "Starting robot without any errors..."
send_cmd "reset"
send_cmd "start"
sleep 3
echo "Robot is tracking the reference circle perfectly."
show_status
echo ""
read -p "Press Enter to continue..."

# Demo 2: Error Induction and Learning
echo ""
echo -e "${GREEN}=== Demo 2: Error Induction and ILC Learning ===${NC}"
echo "Inducing 35% systematic error..."
send_cmd "error 0.35"
sleep 2
echo "Notice the path turns RED and distorts from the green reference."
sleep 5
echo ""
echo "ILC is now learning from each iteration..."
for i in {1..5}; do
    sleep 4
    echo "After ~$((i*4)) seconds:"
    show_status
    echo ""
done
echo "Notice how RMS Error decreases as ILC learns!"
read -p "Press Enter to continue..."

# Demo 3: Different Error Scenarios
echo ""
echo -e "${GREEN}=== Demo 3: Error Scenarios ===${NC}"
send_cmd "reset"

echo ""
echo "3a) Drift Scenario (mild systematic drift)..."
send_cmd "preset drift"
sleep 8
show_status
echo ""

echo "3b) Phase Lag Scenario (timing/phase errors)..."
send_cmd "reset"
send_cmd "preset lag"
sleep 8
show_status
echo ""

echo "3c) Stochastic Noise (random disturbances)..."
send_cmd "reset"
send_cmd "preset noise"
sleep 8
echo "Notice: ILC learns repeatable errors, but random noise remains"
show_status
echo ""
read -p "Press Enter to continue..."

# Demo 4: Shape Morphing
echo ""
echo -e "${GREEN}=== Demo 4: Shape Morphing ===${NC}"
send_cmd "reset"
send_cmd "error 0"
send_cmd "noise off"

echo "Changing to ELLIPSE..."
send_cmd "shape ellipse 1.3 0.8"
sleep 5
send_cmd "error 0.3"
echo "ILC learning ellipse with errors..."
sleep 10
show_status
echo ""

echo "Morphing to SQUARE..."
send_cmd "shape square 1.6"
send_cmd "error 0.25"
sleep 10
show_status
echo ""

echo "Transforming to STAR..."
send_cmd "shape star 1.1 0.45"
send_cmd "error 0.3"
sleep 10
show_status
echo ""
read -p "Press Enter to continue..."

# Demo 5: Parameter Tuning
echo ""
echo -e "${GREEN}=== Demo 5: Parameter Tuning Effects ===${NC}"

echo "Testing LEARNING RATE effects..."
send_cmd "reset"
send_cmd "shape circle 1.0"
send_cmd "error 0.4"

echo ""
echo "5a) Low learning rate (L=0.3, α=0.5) - slow convergence..."
send_cmd "lr 0.3"
send_cmd "smooth 0.5"
echo "   Effective rate: 0.15"
sleep 12
show_status
echo ""

send_cmd "reset"
send_cmd "error 0.4"
echo "5b) High learning rate (L=0.7, α=0.8) - fast convergence..."
send_cmd "lr 0.7"
send_cmd "smooth 0.8"
echo "   Effective rate: 0.56"
sleep 8
show_status
echo ""

echo "Testing SMOOTHING FACTOR effects..."
send_cmd "reset"
send_cmd "error 0.35"
send_cmd "lr 0.6"

echo ""
echo "5c) Low smoothing (α=0.2) - very gradual updates..."
send_cmd "smooth 0.2"
echo "   Effective rate: 0.12"
sleep 12
show_status
echo ""

send_cmd "reset"
send_cmd "error 0.35"
echo "5d) High smoothing (α=0.9) - aggressive updates..."
send_cmd "lr 0.6"
send_cmd "smooth 0.9"
echo "   Effective rate: 0.54"
sleep 8
show_status
echo ""

echo "5e) Optimal balance (L=0.6, α=0.6)..."
send_cmd "reset"
send_cmd "error 0.35"
send_cmd "smooth 0.6"
send_cmd "lr 0.6"
echo "   Effective rate: 0.36"
sleep 10
show_status
echo ""
read -p "Press Enter to continue..."

# Demo 6: Convergence Test
echo ""
echo -e "${GREEN}=== Demo 6: Long-term Convergence Test ===${NC}"
echo "Monitoring ILC convergence over 20 iterations..."
send_cmd "reset"
send_cmd "error 0.4"
send_cmd "lr 0.6"
send_cmd "smooth 0.6"
send_cmd "noise off"
echo "Using effective learning rate: 0.36"

echo ""
echo -e "${CYAN}Iteration | RMS Error${NC}"
echo "------------------------"

for i in {1..20}; do
    sleep 2
    output=$(echo "status" | timeout 2 ./build/ilc_client 2>/dev/null | grep "RMS Error" | awk '{print $3}')
    iter=$(echo "status" | timeout 2 ./build/ilc_client 2>/dev/null | grep "Iteration" | awk '{print $2}')
    printf "%8s | %s\n" "$iter" "$output"
done

echo ""
echo "Notice exponential convergence pattern!"
read -p "Press Enter to finish demo..."

# Cleanup
echo ""
echo -e "${GREEN}=== Demo Complete ===${NC}"
echo "Stopping simulation..."
send_cmd "stop"
send_cmd "reset"

echo ""
echo "Key Takeaways:"
echo "  ✓ ILC automatically learns and corrects systematic errors"
echo "  ✓ Smoothing factor controls update speed (higher = faster)"
echo "  ✓ Effective learning rate = L × α"
echo "  ✓ Works with different shapes and error types"
echo "  ✓ Typical convergence: 10-25 iterations with balanced settings"
echo "  ✓ Converges exponentially to zero error"
echo ""
echo "Try experimenting with your own parameter combinations!"
echo ""
