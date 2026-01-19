#!/bin/bash
# setup.sh - Complete installation and setup for ILC Real-Time Tracker

set -e  # Exit on error

echo "========================================"
echo "  ILC Real-Time Tracker Setup"
echo "========================================"
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check OS
if [[ ! -f /etc/os-release ]]; then
    echo -e "${RED}Error: Cannot detect OS version${NC}"
    exit 1
fi

source /etc/os-release
echo -e "${GREEN}Detected OS: $PRETTY_NAME${NC}"

if [[ "$ID" != "ubuntu" ]]; then
    echo -e "${YELLOW}Warning: This script is designed for Ubuntu 22.04${NC}"
    echo "It may work on other Debian-based systems, but YMMV."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check if running in WSL
if grep -qi microsoft /proc/version; then
    echo -e "${YELLOW}WSL detected${NC}"
    echo "Note: You'll need an X server (VcXsrv, X410, or similar)"
    echo "Make sure DISPLAY is set correctly: export DISPLAY=:0"
fi

echo ""
echo "Step 1: Installing dependencies..."

# Update package list
sudo apt-get update

# Install required packages
PACKAGES="build-essential libx11-dev g++ make"

for pkg in $PACKAGES; do
    if dpkg -l | grep -q "^ii  $pkg"; then
        echo -e "  ${GREEN}✓${NC} $pkg already installed"
    else
        echo -e "  ${YELLOW}Installing $pkg...${NC}"
        sudo apt-get install -y $pkg
    fi
done

echo -e "${GREEN}✓ All dependencies installed${NC}"
echo ""

# Create project structure
echo "Step 2: Creating project structure..."

PROJECT_DIR="ilc_tracker"
if [[ -d "$PROJECT_DIR" ]]; then
    echo -e "${YELLOW}Directory $PROJECT_DIR already exists${NC}"
    read -p "Overwrite? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$PROJECT_DIR"
    else
        echo "Aborting."
        exit 1
    fi
fi

mkdir -p "$PROJECT_DIR"
cd "$PROJECT_DIR"

echo -e "${GREEN}✓ Project directory created: $PROJECT_DIR${NC}"
echo ""

# Create source files
echo "Step 3: Creating source files..."

# Note: In real usage, you'd copy the actual simulator.cpp and client.cpp here
# For this example, we'll just create placeholders
cat > simulator.cpp << 'EOF'
// simulator.cpp should be copied here
// See the artifact "simulator.cpp" for the complete implementation
EOF

cat > client.cpp << 'EOF'
// client.cpp should be copied here
// See the artifact "client.cpp" for the complete implementation
EOF

echo -e "${YELLOW}⚠ You need to copy simulator.cpp and client.cpp to this directory${NC}"
echo "Files should be in the same directory as this script."
echo ""

# Create build script
cat > build.sh << 'BUILDSCRIPT'
#!/bin/bash
set -e

echo "=== Building ILC Simulator ==="

mkdir -p build

echo "Compiling simulator..."
g++ -o build/ilc_simulator simulator.cpp \
    -lX11 -lpthread \
    -std=c++17 -O2 -Wall

echo "Compiling client..."
g++ -o build/ilc_client client.cpp \
    -lpthread \
    -std=c++17 -O2 -Wall

echo ""
echo "=== Build Complete ==="
echo "Executables:"
echo "  build/ilc_simulator"
echo "  build/ilc_client"
BUILDSCRIPT

chmod +x build.sh

# Create run scripts
cat > run_simulator.sh << 'RUNSCRIPT'
#!/bin/bash
if [[ ! -f build/ilc_simulator ]]; then
    echo "Error: Simulator not built. Run ./build.sh first"
    exit 1
fi

echo "Starting ILC Simulator..."
echo "Close window or press Ctrl+C to exit"
./build/ilc_simulator
RUNSCRIPT

chmod +x run_simulator.sh

cat > run_client.sh << 'CLIENTSCRIPT'
#!/bin/bash
if [[ ! -f build/ilc_client ]]; then
    echo "Error: Client not built. Run ./build.sh first"
    exit 1
fi

echo "Connecting to simulator..."
./build/ilc_client
CLIENTSCRIPT

chmod +x run_client.sh

# Create quick test script
cat > quick_test.sh << 'TESTSCRIPT'
#!/bin/bash
echo "=== Quick ILC Test ==="
echo ""
echo "This will:"
echo "1. Start simulation"
echo "2. Induce 30% error"
echo "3. Show status after 10 seconds"
echo "4. Stop simulation"
echo ""
read -p "Press Enter to continue..."

(
    sleep 2
    echo "start"
    sleep 1
    echo "error 0.3"
    echo "Status at start:"
    echo "status"
    sleep 10
    echo "Status after 10 seconds:"
    echo "status"
    echo "stop"
    sleep 1
) | ./build/ilc_client
TESTSCRIPT

chmod +x quick_test.sh

echo -e "${GREEN}✓ Scripts created${NC}"
echo ""

# Create README
cat > README.txt << 'READMEFILE'
ILC Real-Time Tracker
====================

Quick Start:
1. Copy simulator.cpp and client.cpp to this directory
2. Run: ./build.sh
3. Terminal 1: ./run_simulator.sh
4. Terminal 2: ./run_client.sh

Commands in client:
  start          - Start robot
  error 0.3      - Induce 30% error
  lr 0.5         - Set learning rate
  smooth 0.3     - Set smoothing factor
  shape circle   - Change to circle
  shape ellipse  - Change to ellipse
  shape square   - Change to square
  shape star     - Change to star
  preset drift   - Apply drift scenario
  preset lag     - Apply lag scenario
  preset noise   - Apply noise scenario
  status         - Show metrics
  help           - Show all commands
  quit           - Exit

See full README.md for detailed documentation.
READMEFILE

echo -e "${GREEN}✓ README created${NC}"
echo ""

echo "========================================"
echo "  Setup Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo ""
echo "  1. Copy the source files:"
echo "     - simulator.cpp"
echo "     - client.cpp"
echo "     to: $(pwd)"
echo ""
echo "  2. Build the project:"
echo "     cd $(pwd)"
echo "     ./build.sh"
echo ""
echo "  3. Run the simulator:"
echo "     Terminal 1: ./run_simulator.sh"
echo "     Terminal 2: ./run_client.sh"
echo ""
echo "  4. Or try the quick test:"
echo "     ./quick_test.sh"
echo ""
echo -e "${GREEN}Project directory: $(pwd)${NC}"
echo ""
echo "For detailed documentation, see README.md"
echo ""
