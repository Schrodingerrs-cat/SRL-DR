# ILC Real-Time 3D Printer Simulator 

A comprehensive C++ implementation of Iterative Learning Control (ILC) for trajectory tracking with real-time 2D/3D visualization, dynamic shape morphing, dome construction, and G-code generation for 3D printing applications.

## Architecture

**3-Node System:**
1. **Simulator Node** (`simulator.cpp`) - ILC engine, plant simulation, X11 2D visualization, TCP server
2. **3D Renderer** (`renderer3d.cpp`) - OpenGL-based 3D visualization with interactive controls
3. **Command Client** (`client.cpp`) - TCP-based terminal interface for real-time control

**Communication:**
- Port 8765: Command client ↔ Simulator
- Port 8766: Simulator → 3D Renderer (layer streaming)

## Key Features

### Core ILC Functionality
✅ **Adaptive ILC** with smooth reference tracking  
✅ Dynamic reference morphing during runtime (shape-to-shape transitions)  
✅ Automatic correction decay for smooth shape changes  
✅ Multi-harmonic systematic error modeling  
✅ Configurable learning rate and smoothing parameters  
✅ Real-time RMS error monitoring  

### Advanced 3D Capabilities
✅ **Dome Construction** - Adaptive convergence to target shapes with automatic stopping  
✅ **Shape Morphing** - Smooth online transitions between geometries  
✅ **Layer Stacking** - Vertical offset management for 3D structures  
✅ **G-code Export** - Continuous append to `ilc_path_combined.gcode`  
✅ **3D Visualization** - Real-time mesh rendering with rotation, zoom, and pan  

### Dynamic Visualization
✅ **2D View (X11)** - Mouse wheel zoom, grid display, path ghosting  
✅ **3D View (OpenGL)** - Interactive camera controls, layered mesh display  
✅ **Dual rendering** - Synchronized 2D cross-section and 3D build visualization  
✅ **Auto-scaling bounds** - Viewport adapts to current shape dimensions  

### Preset Scenarios
✅ Drift, lag, deform, and noise disturbance profiles  
✅ One-command setup for testing convergence behavior  

## Installation

### Prerequisites (Ubuntu 22.04 / Debian-based)

```bash
sudo apt-get update
sudo apt-get install -y build-essential libx11-dev freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### Build

```bash
chmod +x build.sh
./build.sh
```

This creates:
- `build/ilc_simulator` - Main ILC engine with 2D visualization
- `build/ilc_renderer3d` - 3D OpenGL viewer
- `build/ilc_client` - Command-line interface

## Quick Start

### Terminal 1: Start 3D Renderer (Optional but Recommended)
```bash
./build/ilc_renderer3d
```

You'll see:
- OpenGL window with 3D axes and grid
- Waiting for data on port 8766
- Interactive camera controls active

**3D Controls:**
- **Mouse drag**: Rotate view
- **+/- keys or buttons**: Zoom in/out
- **Space**: Pause/resume animation
- **R**: Reset camera view
- **ESC**: Exit

### Terminal 2: Start Simulator
```bash
./build/ilc_simulator
```

You'll see:
- X11 window with 2D grid and reference circle
- Command server listening on port 8765
- Renderer connection attempt (succeeds if Terminal 1 is running)
- Ready for commands

**2D Controls:**
- **Mouse wheel**: Zoom in/out
- **+/- keys**: Zoom controls
- **R**: Reset view to fit current shape

### Terminal 3: Start Client
```bash
./build/ilc_client
```

Interactive command prompt: `ilc>`

### Basic Workflow

```bash
# Start the simulation
ilc> start

# Induce 30% systematic error
ilc> error 0.3

# Watch ILC adapt over iterations in both 2D and 3D views
# Layers stack vertically in 3D renderer
# 2D view shows current iteration's cross-section

# Check convergence
ilc> status

# Change shape during runtime (smooth morph)
ilc> shape ellipse 1.2 0.8

# Stop and reset
ilc> stop
ilc> reset
```

## Command Reference

### Simulation Control

| Command | Description | Example |
|---------|-------------|---------|
| `start` | Start robot motion and ILC learning | `start` |
| `stop` | Pause simulation | `stop` |
| `reset` | Clear all learning, reset 3D view | `reset` |
| `status` | Show iteration, RMS error, settings | `status` |
| `help` | Display command reference | `help` |
| `quit` | Exit client | `quit` |

### Parameter Tuning

| Command | Range | Description | Example |
|---------|-------|-------------|---------|
| `error <level>` | 0.0 - 1.0 | System error magnitude (0-100%) | `error 0.4` |
| `lr <rate>` | 0.1 - 0.8 | ILC learning rate | `lr 0.5` |
| `smooth <alpha>` | 0.1 - 1.0 | Smoothing factor for correction updates | `smooth 0.3` |
| `noise on\|off` | - | Enable/disable stochastic disturbances | `noise on` |

**Smoothing Factor (α):**
- **0.1-0.3**: Very smooth, slow convergence (good for noisy systems)
- **0.3-0.5**: Balanced (recommended)
- **0.6-1.0**: Fast, may oscillate (use with low learning rate)

**Effective Learning:** Combined effect = `α × learning_rate`

### Shape Commands

Dynamic shape changing with smooth online morphing:

```bash
# Circle with custom radius
ilc> shape circle 1.5

# Ellipse (semi-major axis, semi-minor axis)
ilc> shape ellipse 1.2 0.7

# Square (side length)
ilc> shape square 1.8

# 5-pointed star (outer radius, inner radius)
ilc> shape star 1.0 0.4
```

**Behavior:**
- **Simulation stopped**: Sets new default reference immediately
- **Simulation running**: Initiates smooth morph over ~12 layers with size-matching
  - Generates transition preview in 3D renderer
  - Exports morph to `morph_transition.gcode`
  - ILC corrections decay by 50% to avoid jumps
  - Reference gradually transitions while ILC adapts

### Advanced Shape Operations

#### Dome Construction
Build converging dome structures that adaptively shrink to a target shape:

```bash
# Collapse to a point (zero-dimension dome)
ilc> dome

# Converge to a specific shape
ilc> dome circle 0.3
ilc> dome ellipse 0.8 0.5
ilc> dome square 0.6
ilc> dome star 0.8 0.3
```

**How it works:**
- ILC adaptively morphs reference toward target shape
- Monitors fractional convergence (mean point distance)
- **Automatic stopping** when <5% of initial separation remains
- Corrections freeze after convergence to prevent jitter
- Creates smooth dome/cone structures in 3D view

**Example - Creating a dome:**
```bash
ilc> start
ilc> dome circle 0.2
# Watch in 3D as layers gradually shrink to small circle
# Simulation auto-stops when converged
# Result: Dome structure with smooth taper
```

#### Morph Transition (Legacy)
Gradual interpolation between current and target shape:

```bash
# Morph 50% toward ellipse
ilc> morph ellipse 0.5

# Full transition to star
ilc> morph star 1.0
```

**Note:** For runtime shape changes, use `shape <type>` instead (more sophisticated).

### Preset Scenarios

Quick-start disturbance profiles:

```bash
# Mild drift error (25%)
ilc> preset drift

# Phase lag error (45%)
ilc> preset lag

# Severe deformation (65%)
ilc> preset deform

# Stochastic noise (40% + random disturbances)
ilc> preset noise
```

## Visualization

### 2D View (X11 Window)

**Visual Elements:**
- **Blue dashed line**: Reference trajectory (desired path)
- **Blue solid line**: Current tracked path (robot trajectory)
- **Faded blue paths**: Previous iteration history (ghosting)
- **Blue/yellow dot**: Robot position (real-time)
- **Grid**: World coordinate system with auto-scaling

**Color Coding:**
- Normal operation: Blue tones
- Error induced: Paths show distortion from reference

**Status Display:**
```
Iteration: 12 | RMS Error: 0.0234 | Error Level: 30%
```

### 3D View (OpenGL Renderer)

**Visual Elements:**
- **RGB Axes**: X (red), Y (green), Z (blue)
- **Layered mesh**: Stacked rings with gradient shading
- **Triangle strips**: Connect adjacent layers
- **Ring outlines**: Visible layer boundaries
- **Yellow point**: Current build position cursor

**Shading:**
- Base color: Greyish-blue (#5c708a)
- Gradient fade: Darker toward top layers
- Slight variation: Triangle strips have subtle depth

**Camera:**
- Auto-centers on geometry bounding box
- Distance scales with structure size
- Smooth rotation and zoom

### What to Watch

#### 2D Evolution:
1. **Iteration 0**: Perfect circle (no error)
2. **Error induced**: Path distorts, deviates from reference
3. **ILC learning**: Over 5-10 iterations, path returns to circle
4. **Shape change**: Smooth morph visible as reference updates
5. **Convergence**: RMS error → 0, path matches reference

#### 3D Structure:
1. **Layer stacking**: Each iteration adds a new ring vertically
2. **Dome formation**: Rings progressively shrink toward target
3. **Smooth surfaces**: Triangle mesh shows continuous geometry
4. **Morph transitions**: Visible shape interpolation in vertical structure

## Algorithm Details

### Standard ILC with Adaptive Reference

**Core Update Law:**
```
e_k(t) = reference(t) - actual_k(t)
u_{k+1}(t) = u_k(t) + α × L × e_k(t)
```

Where:
- `reference(t)` = desired trajectory (may update during runtime)
- `actual_k(t)` = robot's achieved position
- `e_k(t)` = tracking error
- `u_k(t)` = feedforward correction at time t, iteration k
- `L` = learning rate (0.1-0.8)
- `α` = smoothing factor (0.1-1.0)

**Correction Limiting:**
```
||correction(t)|| ≤ 1.0  (clipped to prevent instability)
```

### Dynamic Reference Handling

**Shape Change Strategy:**
1. Compute size-match: `scale = radius(current) / radius(new)`
2. Apply scale to new shape (prevents shrinkage)
3. Generate 12-layer morph preview → 3D renderer + G-code
4. **Immediate reference replacement** (green outline jumps)
5. **Decay corrections by 50%** to reduce transient overshoot
6. ILC adapts corrections over next iterations

**Why this works:**
- Old corrections were tuned for old shape
- New shape has different error profile
- Decay prevents abrupt jumps in control input
- ILC quickly relearns optimal corrections for new geometry

### Dome Convergence Algorithm

**Adaptive Morphing:**
```
For each iteration:
    1. Compute distance: d = ||reference - target||
    2. Calculate fractional progress: f = d / d_initial
    3. Adaptive step: step = α × L × clamp(0.8×f, 0.02, 0.25)
    4. Update: reference += step × (target - reference)
    5. If f < 0.05 or maxDistance < ε:
        - Stop morphing
        - Freeze corrections (prevent jitter)
        - Auto-stop simulation
```

**Why adaptive step:**
- Fast convergence when far from target (large f)
- Slow, careful approach near target (small f)
- Prevents overshoot and oscillation
- Automatic convergence detection

### Plant Model

Multi-harmonic systematic errors simulate realistic disturbances:

```cpp
radial_error = ε × (0.25×sin(3θ) + 0.15×sin(5θ) + 0.10×cos(2θ))
phase_error = ε × (0.18 + 0.02×sin(0.5θ + iteration))
contraction = ε × (0.12 + 0.03×cos(1.5θ))
```

**Optional noise:**
```cpp
if (noise_enabled):
    output += N(0, ε×0.04)  // Gaussian random disturbance
```

**Effects:**
- **Radial**: Bumps and valleys in shape
- **Phase**: Angular lag/lead (rotation-like distortion)
- **Contraction**: Overall shrinking tendency
- **Noise**: Non-repeatable stochastic component

### 3D Layer Management

**Vertical Stacking:**
```cpp
z = layerIndex × layerHeight  (default: 0.1 units)
```

Each completed ILC iteration → new horizontal ring at incremented z-height.

**Throttled Transmission:**
- Only newly completed layers sent to renderer
- Reduces TCP traffic (important for high iteration counts)
- Partial current trajectory shown as top layer preview

**G-code Generation:**
- Continuous append mode to `ilc_path_combined.gcode`
- Only finalized layers written (not in-progress trajectory)
- Standard format: G1 moves with extrusion calculation

## G-code Output

### File Generation

**Primary file:** `ilc_path_combined.gcode`
- Appended continuously during simulation
- Contains all completed layers
- Standard 3D printer format (G1, G0 commands)

**Morph transitions:** `morph_transition.gcode`
- Generated when shape changes during runtime
- Contains 12-layer interpolation preview
- Standalone file (not appended to combined)

### G-code Structure

```gcode
; Generated by ILC simulator
G21 ; millimeters
G90 ; absolute positioning
G92 E0 ; reset extruder
G1 F1200 ; set feedrate

; Layer moves
G0 X1.000 Y0.000 Z0.000 ; rapid to start
G1 X0.995 Y0.100 Z0.000 E0.002 ; extrude
G1 X0.980 Y0.199 Z0.000 E0.004
...

; Shutdown
M104 S0 ; extruder off
M140 S0 ; bed off
M84 ; motors off
```

**Extrusion calculation:**
```
E_total += distance × 0.02  (default factor)
```

### Using G-code

**Compatible slicers:**
- PrusaSlicer (import as post-processed)
- Simplify3D
- Cura (with manual header adjustment)

**Recommended workflow:**
1. Run ILC simulation to completion
2. Open `ilc_path_combined.gcode` in text editor
3. Verify start G-code (temps, homing, etc.)
4. Adjust feedrate `F` value if needed
5. Import into slicer for preview
6. Send to printer or save to SD card

**Safety notes:**
- ⚠️ Always preview G-code before printing
- ⚠️ Verify Z-heights match your printer's range
- ⚠️ Check extrusion multiplier is reasonable
- ⚠️ Add proper start/end G-code for your printer

## Example Sessions

### Session 1: Basic ILC Learning with 3D Visualization

```bash
# Terminal 1
./build/ilc_renderer3d

# Terminal 2
./build/ilc_simulator

# Terminal 3
./build/ilc_client
ilc> start
ilc> error 0.35
```

**What you'll see:**
- **2D view**: Red distorted circle gradually corrects to green reference
- **3D view**: Layers stack vertically, early layers show distortion, later layers converge
- **RMS error**: Drops from ~0.15 to <0.01 over 8-12 iterations
- **Status**: `Iteration: 10 | RMS Error: 0.0089 | Error Level: 35%`

```bash
ilc> stop
ilc> status
# Verify convergence achieved
```

### Session 2: Dynamic Shape Morphing

```bash
ilc> start
# Let it run for 5 iterations to establish baseline
ilc> shape ellipse 1.3 0.8
```

**What happens:**
1. Command triggers size-match calculation
2. 12-layer morph preview generated → 3D renderer
3. `morph_transition.gcode` written
4. 2D reference (green) jumps to ellipse shape
5. Corrections decay 50%
6. ILC begins tracking new shape
7. Over next 8 iterations, path converges to ellipse

```bash
ilc> shape star 1.1 0.5
# Smooth transition from ellipse → star
# 3D structure shows both geometries in vertical stack
ilc> stop
```

### Session 3: Dome Construction

```bash
ilc> reset
ilc> start
ilc> dome circle 0.3
```

**Observation:**
- **2D view**: Reference circle shrinks gradually each iteration
- **3D view**: Cone/dome structure forms with smooth taper
- **Automatic stop**: Simulation halts when <5% from target
- **Console**: `[ILC] Dome convergence complete (remaining=0.042, maxDist=0.008)`

```bash
ilc> status
# Iteration: 18 (auto-stopped)
# RMS Error: 0.0012
# Corrections frozen: YES
```

**Advanced dome:**
```bash
ilc> reset
ilc> start
ilc> shape ellipse 1.5 0.9
# Build up base layers
ilc> dome star 0.4 0.15
# Creates ellipse→star morphing dome
# Final structure: elliptical base tapering to star tip
```

### Session 4: Parameter Sensitivity Study

```bash
# Test 1: High learning rate, low smoothing
ilc> reset
ilc> lr 0.75
ilc> smooth 0.2
ilc> start
ilc> error 0.4
# Result: Slow but very smooth convergence (~15 iterations)

ilc> stop
ilc> reset

# Test 2: Balanced settings
ilc> lr 0.5
ilc> smooth 0.5
ilc> start
ilc> error 0.4
# Result: Moderate speed, stable (~10 iterations)

ilc> stop
ilc> reset

# Test 3: Aggressive settings
ilc> lr 0.8
ilc> smooth 0.9
ilc> start
ilc> error 0.4
# Result: Fast but may oscillate slightly (~6 iterations with overshoot)
```

### Session 5: Preset Scenarios

```bash
ilc> preset deform
# Applies 65% error (severe distortion)
ilc> start
# Watch ILC handle extreme multi-harmonic errors
# Convergence takes 12-15 iterations

ilc> stop
ilc> preset noise
# 40% error + random disturbances
ilc> start
# ILC learns repeatable component
# Small residual noise remains (non-repeatable)
```

### Session 6: Complex 3D Structure

```bash
ilc> reset
ilc> start
# Build 10 circle layers
ilc> shape square 1.8
# 10 more square layers
ilc> dome circle 0.2
# Dome top

# Final structure:
# - Cylindrical base (circle)
# - Cube middle section (square)
# - Conical dome top (shrinking circle)
```

## Performance

### System Requirements
- **CPU**: 2+ cores, <5% usage per node
- **RAM**: ~30MB total (all processes)
- **GPU**: Any OpenGL 2.0+ compatible (integrated graphics sufficient)
- **Network**: Localhost TCP (minimal overhead)

### Performance Metrics
- **2D render rate**: 30 FPS (throttled to reduce flicker)
- **3D render rate**: 60 FPS
- **ILC update rate**: ~60 Hz (16ms loop)
- **TCP latency**: <1ms (local)
- **Layer transmission**: ~5ms per ring (150 points)
- **G-code write**: Asynchronous, no blocking

### Scalability
- **NUM_POINTS**: Currently 150, tested up to 500 (smooth)
- **Layers**: Renderer handles 200+ layers efficiently
- **Memory**: Linear growth with layer count (~100KB per 10 layers)

### Optimization Tips
```cpp
// In simulator.cpp
const int NUM_POINTS = 100;  // Reduce for faster updates

// Reduce 2D render frequency (line ~2800)
if (msSince >= 50) { // 20 FPS instead of 30

// Batch G-code writes
static int gcodeWriteCounter = 0;
if (++gcodeWriteCounter % 5 == 0) {
    writeGCodeForPath(...);
}
```

## Troubleshooting

### 3D Renderer Issues

**"Cannot connect to X server"**
```bash
echo $DISPLAY  # Should show :0 or :1
export DISPLAY=:0
# For WSL2: Install VcXsrv or X410, enable OpenGL
```

**Black screen / no rendering**
```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"
# Should show 2.0 or higher

# Update graphics drivers
sudo ubuntu-drivers autoinstall
```

**Renderer not receiving data**
```bash
# Check port availability
netstat -tuln | grep 8766

# Verify renderer started first
ps aux | grep renderer3d

# Check simulator connection log
# Should see: [RendererClient] Connected to renderer on port 8766
```

### Simulator Issues

**"Bind failed" on port 8765**
```bash
# Kill existing process
pkill ilc_simulator

# Or use different port (edit SERVER_PORT in simulator.cpp)
```

**Jerky 2D rendering**
```bash
# Increase render interval (line ~2800 in simulator.cpp)
if (msSince >= 50) { // Reduce from 33ms to 50ms
```

**ILC not converging**
```bash
# Check parameters
ilc> status
# Verify learning rate and smoothing are reasonable

# Try preset
ilc> reset
ilc> lr 0.5
ilc> smooth 0.4
ilc> preset drift
ilc> start
```

### G-code Issues

**File not created**
```bash
# Check write permissions
ls -la ilc_path_combined.gcode

# Verify simulator is running and simulation started
# G-code only writes after iterations complete
```

**Corrupted G-code**
```bash
# Kill simulator cleanly (not Ctrl+C during write)
# Reopen file in append mode safely (simulator handles this)

# Validate G-code
cat ilc_path_combined.gcode | grep "G1"
# Should see continuous G1 commands
```

## Advanced Modifications

### Custom Shapes

Add to `ShapeGenerator` class in `simulator.cpp`:

```cpp
static std::vector<Point2D> generateHeart(int numPoints, double scale = 1.0) {
    std::vector<Point2D> points;
    for (int i = 0; i < numPoints; i++) {
        double t = (2.0 * M_PI * i) / numPoints - M_PI;
        double x = 16 * pow(sin(t), 3);
        double y = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
        points.push_back(Point2D(x*scale/20.0, y*scale/20.0));
    }
    return points;
}
```

Register in command handler (~line 1800):
```cpp
else if (shapeType == "heart") {
    double scale = 1.0;
    iss >> scale;
    newRef = ShapeGenerator::generateHeart(NUM_POINTS, scale);
}
```

Update client help text in `client.cpp`:
```cpp
std::cout << "      heart [scale]        - Heart shape\n";
std::cout << "                             Example: shape heart 1.2\n";
```

### Tune Plant Dynamics

Edit `plantModel()` method (~line 600):

```cpp
// Add 4th harmonic distortion
double radialError = systemErrorLevel * (
    0.25 * std::sin(3 * theta) +
    0.15 * std::sin(5 * theta) +
    0.10 * std::cos(2 * theta) +
    0.08 * std::sin(4 * theta)  // NEW
);

// Increase phase lag
double phaseError = systemErrorLevel * (0.25 + 0.03 * std::sin(0.5 * theta + iteration));
```

### Custom Layer Height

Change default stacking offset (~line 17):
```cpp
static constexpr double DEFAULT_LAYER_HEIGHT = 0.05; // Finer layers
```

### Persistent Storage Between Runs

Add to `ILCController` class:
```cpp
void saveCorrections(const std::string& filename) {
    std::ofstream ofs(filename);
    for (const auto& c : corrections) {
        ofs << c.x << " " << c.y << " " << c.z << "\n";
    }
    ofs.close();
}

void loadCorrections(const std::string& filename) {
    std::ifstream ifs(filename);
    corrections.clear();
    double x, y, z;
    while (ifs >> x >> y >> z) {
        corrections.push_back(Point3D(x, y, z));
    }
}
```

Add commands in `processCommand()`:
```cpp
else if (action == "save") {
    std::string file;
    iss >> file;
    ilc->saveCorrections(file);
    return "OK: Corrections saved to " + file + "\n";
}
else if (action == "load") {
    std::string file;
    iss >> file;
    ilc->loadCorrections(file);
    return "OK: Corrections loaded from " + file + "\n";
}
```

### Multi-Material Printing

Modify G-code generator to support tool changes:

```cpp
void writeGCodeForPath(const std::vector<Point3D>& path, std::ofstream& ofs, int toolNumber = 0) {
    if (toolNumber > 0) {
        ofs << "T" << toolNumber << " ; Switch to tool " << toolNumber << "\n";
        ofs << "G92 E0 ; Reset extruder\n";
    }
    // ... rest of G-code generation
}
```

## Theory & Convergence

### ILC Convergence Theorem

For linear time-invariant plant G(s), ILC converges if:
```
||I - L×G|| < 1
```

Where:
- I = identity
- L = learning gain matrix
- G = plant transfer function

**Practical implications:**
- Learning rate L ∈ (0, 2/||G||) ensures convergence
- Smoothing factor α slows update → improves robustness
- Multi-pass nature: Perfect tracking achievable for repeatable disturbances

### Adaptive Reference Extension

**Novel contribution:** This implementation extends classical ILC to handle:
1. **Time-varying reference** (shape morphing)
2. **Correction decay** (transient management)
3. **Adaptive convergence** (dome construction)

**Key insight:** When reference changes, old corrections create transient error. Solution:
- Decay corrections by factor γ ∈ [0,1]
- ILC relearns in ~5-8 iterations (faster than from scratch)
- Combined with size-matching → smooth transitions

### Dome Convergence Analysis

**Morphing step:**
```
r_{k+1} = r_k + β(k) × (r_target - r_k)
```

Where adaptive step:
```
β(k) = α × L × clamp(0.8 × f(k), 0.02, 0.25)
f(k) = ||r_k - r_target|| / ||r_0 - r_target||
```

**Convergence:**
- Monotonic decrease in ||r_k - r_target||
- Automatic stopping at 5% threshold
- Geometric convergence rate (exponential approach)

## Video Implementation

You can find the video implementation here in this playlist:
[ILC 3D Printer Simulator - Video Tutorials](https://www.youtube.com/playlist?list=PLjOq_2Ap5hLZfs79zW1JtutaKBr80EMHk)

## Citing This Work

```bibtex
@software{ilc_3d_printer_simulator,
  title={ILC 3D Printer Simulator: Adaptive Learning Control with Dynamic Shape Morphing},
  author={Aryan Shah},
  year={2025},
  url={[https://github.com/yourusername/ilc-simulator](https://github.com/Schrodingerrs-cat/SRL-DR)},
  note={C++ implementation with OpenGL 3D visualization and G-code generation}
}
```

## Future Enhancements

**Planned features:**
- [ ] Multi-extruder support (IDEX systems)
- [ ] Temperature-dependent error modeling
- [ ] Real-time slicing integration
- [ ] On-printer execution
- [ ] CSV data export for analysis
- [ ] Automated convergence plots

**Community contributions welcome!**

## License

MIT License - Free for research, educational, and commercial use.

## Contact & Support

- **Issues**: Open GitHub issue with logs and screenshots
- **Questions**: Discussion board or email maintainer
- **Contributions**: Pull requests welcome (see CONTRIBUTING.md)
