#!/bin/bash
# =====================================================================
# build.sh — Build script for ILC simulator project
# Components:
#   1. ilc_simulator  -> Core + ILC + 2D X11 visualization
#   2. ilc_client     -> Command interface (for user input/control)
#   3. ilc_renderer3d -> Optional 3D OpenGL visualizer (spiral printing)
# =====================================================================

set -e  # Exit on any error

echo "=== Building ILC Simulator Project ==="

# ---------------------------------------------------------------------
# 0. Ensure dependencies
# ---------------------------------------------------------------------
echo "[CHECK] Verifying required system packages..."

# Check X11 dev libs for simulator
if ! dpkg -l | grep -q libx11-dev; then
    echo "Installing X11 development libraries..."
    sudo apt-get update
    sudo apt-get install -y libx11-dev
fi

# Check OpenGL dev libs if renderer exists
if [ -f ../renderer3d.cpp ]; then
    if ! dpkg -l | grep -q freeglut3-dev; then
        echo "Installing OpenGL/GLUT development libraries..."
        sudo apt-get update
        sudo apt-get install -y freeglut3-dev libglu1-mesa-dev mesa-common-dev
    fi
fi

# ---------------------------------------------------------------------
# 1. Create build directory
# ---------------------------------------------------------------------
mkdir -p build
cd build

# ---------------------------------------------------------------------
# 2. Build simulator (core + 2D visualization)
# ---------------------------------------------------------------------
echo ""
echo "[BUILD] Compiling simulator (2D + ILC core)..."
g++ -o ilc_simulator ../simulator.cpp \
    -lX11 -lpthread \
    -std=c++17 -O2 -Wall \
    && echo "ilc_simulator built successfully." \
    || { echo "Simulator compilation failed!"; exit 1; }

# ---------------------------------------------------------------------
# 3. Build command-line client
# ---------------------------------------------------------------------
echo ""
echo "[BUILD] Compiling command client..."
g++ -o ilc_client ../client.cpp \
    -lpthread \
    -std=c++17 -O2 -Wall \
    && echo "ilc_client built successfully." \
    || { echo "Client compilation failed!"; exit 1; }

# ---------------------------------------------------------------------
# 4. Optionally build renderer (OpenGL 3D visualizer)
# ---------------------------------------------------------------------
if [ -f ../renderer3d.cpp ]; then
    echo ""
    echo "[BUILD] Found renderer3d.cpp — compiling 3D renderer with OpenGL..."
    g++ -o ilc_renderer3d ../renderer3d.cpp \
        -lGL -lGLU -lglut -lpthread \
        -std=c++17 -O2 -Wall \
        && echo "ilc_renderer3d built successfully." \
        || { echo "Renderer compilation failed!"; exit 1; }
else
    echo ""
    echo "[SKIP] renderer3d.cpp not found — skipping 3D renderer build."
fi

# ---------------------------------------------------------------------
# 5. Summary
# ---------------------------------------------------------------------
echo ""
echo "=== Build Complete ==="
echo "Executables created in ./build/:"
echo "  - ilc_simulator   -> 2D X11 visualizer + ILC logic"
echo "  - ilc_client      -> Command interface"
if [ -f ilc_renderer3d ]; then
    echo "  - ilc_renderer3d  -> 3D OpenGL spiral visualizer"
fi
echo ""
echo "Run Instructions:"
echo "  Terminal 1: ./build/ilc_simulator"
echo "  Terminal 2: ./build/ilc_client"
if [ -f ilc_renderer3d ]; then
    echo "  Terminal 3: ./build/ilc_renderer3d"
fi
echo ""

