#!/bin/bash
# build.sh - Build the ILC simulator and client

echo "=== Building ILC Simulator ==="

# Check dependencies
if ! dpkg -l | grep -q libx11-dev; then
    echo "Installing X11 development libraries..."
    sudo apt-get update
    sudo apt-get install -y libx11-dev
fi

# Create build directory
mkdir -p build
cd build

# Build simulator
echo "Compiling simulator..."
g++ -o ilc_simulator ../simulator.cpp \
    -lX11 -lpthread \
    -std=c++17 -O2 -Wall \
    || { echo "Simulator compilation failed!"; exit 1; }

# Build client
echo "Compiling client..."
g++ -o ilc_client ../client.cpp \
    -lpthread \
    -std=c++17 -O2 -Wall \
    || { echo "Client compilation failed!"; exit 1; }

echo ""
echo "=== Build Complete ==="
echo "Executables created in build/ directory:"
echo "  - ilc_simulator (visualization + ILC engine)"
echo "  - ilc_client (command interface)"
echo ""
echo "To run:"
echo "  Terminal 1: ./build/ilc_simulator"
echo "  Terminal 2: ./build/ilc_client"
echo ""
