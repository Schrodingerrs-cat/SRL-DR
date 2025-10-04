# ILC Real-Time Tracker - C++ Implementation

A complete C++ implementation of Iterative Learning Control (ILC) for trajectory tracking with real-time visualization and command interface.

## Architecture

**2-Node System:**
1. **Simulator Node** (`simulator.cpp`) - ILC engine, robot simulation, X11 visualization
2. **Command Client** (`client.cpp`) - TCP-based terminal interface for control

Communication: TCP socket on port 8765 (localhost)

## Features

✅ **Standard ILC** trajectory tracking with smooth correction  
✅ Error calculated directly from fixed reference (not previous iteration)  
✅ Smoothing factor prevents zig-zagging during learning  
✅ Multiple reference shapes (circle, ellipse, square, star)  
✅ Configurable system errors and noise  
✅ Live X11 visualization  
✅ Terminal-based command interface  
✅ Preset error scenarios  

## Installation

### Prerequisites (Ubuntu 22.04)

```bash
sudo apt-get update
sudo apt-get install -y build-essential libx11-dev
```

### Build

```bash
chmod +x build.sh
./build.sh
```

This creates:
- `build/ilc_simulator` - Main visualization and ILC engine
- `build/ilc_client` - Command interface

## Quick Start

### Terminal 1: Start Simulator
```bash
./build/ilc_simulator
```

You'll see:
- X11 window opens with grid and reference circle
- Server starts on port 8765
- Waiting for commands

### Terminal 2: Start Client
```bash
./build/ilc_client
```

Interactive command prompt appears: `ilc>`

### Basic Workflow

```bash
# Start the robot moving
ilc> start

# Induce 30% systematic error
ilc> error 0.3

# Watch ILC learn and correct over iterations

# Check status
ilc> status

# Stop simulation
ilc> stop

# Reset learning
ilc> reset
```

## Command Reference

### Simulation Control

| Command | Description | Example |
|---------|-------------|---------|
| `start` | Start robot motion | `start` |
| `stop` | Pause simulation | `stop` |
| `reset` | Clear all ILC learning | `reset` |
| `status` | Show current metrics | `status` |

### Parameters

| Command | Range | Description | Example |
|---------|-------|-------------|---------|
| `error <level>` | 0.0 - 1.0 | System error magnitude | `error 0.4` |
| `lr <rate>` | 0.1 - 0.8 | ILC learning rate | `lr 0.5` |
| `smooth <alpha>` | 0.1 - 1.0 | Smoothing factor* | `smooth 0.3` |
| `noise on\|off` | - | Random disturbances | `noise on` |

**Smoothing factor controls gradual correction:**
- Lower (0.1-0.3): Slow, very smooth convergence
- Medium (0.3-0.5): Balanced
- Higher (0.6-1.0): Faster, may oscillate

### Shape Commands

Change reference trajectory on-the-fly:

```bash
# Circle with custom radius
ilc> shape circle 1.5

# Ellipse with semi-major/minor axes
ilc> shape ellipse 1.2 0.7

# Square with side length
ilc> shape square 1.8

# 5-pointed star with outer/inner radius
ilc> shape star 1.0 0.4
```

**Note:** Shape change automatically resets ILC learning.

### Preset Scenarios

Quick test cases:

```bash
# Mild systematic drift
ilc> preset drift

# Phase lag error
ilc> preset lag

# Severe deformation with harmonics
ilc> preset deform

# Stochastic noise
ilc> preset noise
```

## Example Sessions

### Session 1: Basic ILC Learning

```bash
# Terminal 2 (client)
ilc> start
ilc> error 0.3
# Watch robot path turn red and distorted
# Over 5-10 iterations, path gradually returns to green circle
ilc> status
# Iteration: 8
# RMS Error: 0.023451
# Error Level: 30%
ilc> stop
```

### Session 2: Shape Morphing

```bash
ilc> start
ilc> shape ellipse 1.5 0.8
# Robot now tracks ellipse
ilc> error 0.25
# ILC learns ellipse tracking
ilc> shape star 1.2 0.5
# Smoothly transitions to star shape
ilc> stop
```

### Session 3: Parameter Tuning

```bash
ilc> start
ilc> error 0.4
ilc> lr 0.7
ilc> smooth 0.2
# Very smooth, gradual correction
ilc> smooth 0.8
# Faster but may oscillate
ilc> smooth 0.4
# Balanced - watch difference in convergence
```

### Session 4: Noise Handling

```bash
ilc> start
ilc> preset noise
# Error + random disturbances
# ILC learns repeatable part, noise remains
ilc> noise off
ilc> error 0.3
# Pure systematic error - ILC converges perfectly
```

## Visualization

### Colors
- **Green dashed line**: Reference trajectory (desired path)
- **Blue solid line**: Robot path (no error) or learned trajectory
- **Red solid line**: Distorted path (with system error)
- **Faded paths**: Previous iterations (ghosting effect)
- **Orange**: ILC correction vectors (when visible)
- **Blue/Red dot**: Robot position (real-time)

### What to Watch

1. **Initial path** (iteration 0): Follows reference perfectly
2. **After error induced**: Path distorts (red), deviates from green reference
3. **ILC learning**: Over iterations, red path gradually aligns with green
4. **Convergence**: RMS error decreases, path returns to circle
5. **Smooth corrections**: No zig-zagging due to smoothing factor

## Algorithm Details

### Standard ILC Update Law (with Smoothing)

Classic ILC corrects based on the **fixed reference trajectory**:

```
e_k(t) = reference(t) - actual_k(t)
u_{k+1}(t) = u_k(t) + α * L * e_k(t)
```

Where:
- `reference(t)` = desired trajectory point at time t (FIXED)
- `actual_k(t)` = robot's actual position at time t, iteration k
- `e_k(t)` = tracking error (difference from reference)
- `u_k(t)` = control correction at time t, iteration k
- `L` = learning rate (0.1-0.8) - how much to trust the error signal
- `α` = smoothing factor (0.1-1.0) - how fast to update corrections

**Effective learning rate = α × L**
- For fast convergence: Use L=0.8, α=0.8 → effective rate = 0.64
- For smooth convergence: Use L=0.5, α=0.3 → effective rate = 0.15
- For balanced: Use L=0.6, α=0.5 → effective rate = 0.30

**Key point:** Error is always measured against the **reference**, not the previous iteration. The correction is applied in full, but the UPDATE to the correction is smoothed by α to prevent oscillations.

### Plant Model

Multi-harmonic systematic errors:
```
radial_error = ε * (0.25*sin(3θ) + 0.15*sin(5θ) + 0.10*cos(2θ))
phase_error = ε * (0.18 + 0.02*sin(0.5θ + k))
contraction_error = ε * (0.12 + 0.03*cos(1.5θ))
```

Plus optional random noise: `N(0, ε*0.04)`

This creates realistic distortions: drift, lag, deformation, oscillation.

### Error Calculation

Standard ILC computes error directly from reference:
```
For each time point t:
    error(t) = reference(t) - actual(t)
    
No phase matching or trajectory search - direct point-to-point comparison.
```

This is the classical ILC approach where the reference trajectory is the fixed target, and we learn the feedforward correction needed to track it perfectly despite systematic disturbances.

## Troubleshooting

### "Cannot open X display"
```bash
# Check DISPLAY variable
echo $DISPLAY
# Should show :0 or :1

# If empty, set it
export DISPLAY=:0

# For WSL2 users, install X server (VcXsrv, X410)
```

### "Connection failed"
```bash
# Make sure simulator is running first
# Check if port 8765 is available
netstat -an | grep 8765

# Kill existing process if needed
pkill ilc_simulator
```

### Simulator crashes
```bash
# Rebuild with debug symbols
g++ -g -o ilc_simulator simulator.cpp -lX11 -lpthread -std=c++17
gdb ./ilc_simulator
```

### Slow rendering
```bash
# Reduce update rate (edit simulator.cpp)
# Line: if (elapsed >= 16)  // 60 FPS
# Change to: if (elapsed >= 33)  // 30 FPS
```

## Advanced Usage

### Scripted Commands

Create a command script:

```bash
# commands.txt
start
error 0.3
# Wait for learning
status
shape ellipse 1.2 0.8
error 0.4
status
stop
```

Run with:
```bash
./build/ilc_client < commands.txt
```

### Automated Testing

```bash
#!/bin/bash
# test_convergence.sh

echo "start" | ./build/ilc_client
sleep 1

echo "error 0.3" | ./build/ilc_client

# Monitor for 20 iterations
for i in {1..20}; do
    sleep 2
    echo "status" | ./build/ilc_client | grep "RMS Error"
done

echo "stop" | ./build/ilc_client
```

### Data Logging

Modify `simulator.cpp` to log metrics:

```cpp
// In completeIteration()
std::ofstream log("ilc_data.csv", std::ios::app);
log << iteration << "," << rmsError << "," << systemErrorLevel << "\n";
log.close();
```

## Performance

- **Update rate**: ~60 FPS
- **ILC points**: 150 (configurable in code)
- **TCP latency**: <1ms (localhost)
- **Memory**: ~10MB

### Change Number of Points

Edit `simulator.cpp`:
```cpp
const int NUM_POINTS = 200;  // Increase for smoother paths
```

### Add New Shapes

In `ShapeGenerator` class:
```cpp
static std::vector<Point2D> generateHeart(int numPoints) {
    std::vector<Point2D> points;
    for (int i = 0; i < numPoints; i++) {
        double t = (2.0 * M_PI * i) / numPoints;
        double x = 16 * pow(sin(t), 3);
        double y = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
        points.push_back(Point2D(x/20.0, y/20.0));
    }
    return points;
}
```


### Tune Plant Model

Edit `plantModel()` in `ILCController`:
```cpp
// Adjust harmonic weights
double radialError = systemErrorLevel * (
    0.30 * std::sin(2 * theta) +  // Dominant 2nd harmonic
    0.20 * std::sin(4 * theta)    // Add 4th harmonic
);
```

## Theory Reference

This implements the classical ILC update law:

```
u_{k+1} = u_k + L * e_k
```

With enhancements:
1. **Smoothing factor** α: Prevents oscillation
2. **Nearest-neighbor matching**: Handles phase errors
3. **Correction limiting**: Prevents instability
4. **Multi-harmonic errors**: Realistic disturbances

Convergence condition (sufficient):
```
||1 - L*G|| < 1
```

Where G is the plant transfer function. Learning rate L must be chosen appropriately.
