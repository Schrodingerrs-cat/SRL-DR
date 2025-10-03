# ILC Real-Time Tracker - Quick Reference

## Installation (One-time)
```bash
# Ubuntu 22.04
sudo apt-get update && sudo apt-get install -y build-essential libx11-dev

# Build
make
# or
./build.sh
```

## Running

### Method 1: Using Make
```bash
# Terminal 1
make run-sim

# Terminal 2
make run-client
```

### Method 2: Direct
```bash
# Terminal 1
./build/ilc_simulator

# Terminal 2
./build/ilc_client
```

## Essential Commands

| Command | What It Does | Example |
|---------|--------------|---------|
| `start` | Begin robot motion | `start` |
| `stop` | Pause simulation | `stop` |
| `reset` | Clear all learning | `reset` |
| `error 0.3` | Induce 30% error | `error 0.3` |
| `lr 0.5` | Set learning rate | `lr 0.5` |
| `smooth 0.3` | Set smoothing | `smooth 0.3` |
| `status` | Show metrics | `status` |

## Common Workflows

### 1. Basic ILC Test
```bash
ilc> start
ilc> error 0.3
# Wait and watch convergence
ilc> status
ilc> stop
```

### 2. Shape Change
```bash
ilc> start
ilc> shape ellipse 1.2 0.8
ilc> error 0.25
# Watch ILC learn new shape
```

### 3. Parameter Sweep
```bash
ilc> start
ilc> error 0.4
ilc> lr 0.3    # Try different rates
ilc> smooth 0.2
ilc> lr 0.6
ilc> smooth 0.5
```

## Presets (Quick Scenarios)

| Preset | Description | Command |
|--------|-------------|---------|
| Drift | Mild systematic drift | `preset drift` |
| Lag | Phase/timing errors | `preset lag` |
| Deform | Severe deformation | `preset deform` |
| Noise | Random disturbances | `preset noise` |

## Shapes

| Shape | Command | Parameters |
|-------|---------|------------|
| Circle | `shape circle` | `[radius]` default=1.0 |
| Ellipse | `shape ellipse` | `[a] [b]` default=1.2, 0.7 |
| Square | `shape square` | `[side]` default=1.6 |
| Star | `shape star` | `[outer] [inner]` default=1.0, 0.4 |

## Parameter Ranges

| Parameter | Range | Recommended | Effect |
|-----------|-------|-------------|--------|
| Error Level | 0.0-1.0 | 0.2-0.5 | Distortion magnitude |
| Learning Rate | 0.1-0.8 | 0.5-0.7 | How much to trust error |
| Smoothing | 0.1-1.0 | 0.5-0.8 | Update speed (higher = faster) |

**Note:** Effective learning = L × α. For fast convergence, use both high!

## Visual Indicators

| Color | Meaning |
|-------|---------|
| 🟢 Green dashed | Reference (desired path) |
| 🔵 Blue solid | Robot path (no error) |
| 🔴 Red solid | Distorted path (with error) |
| 🟠 Orange dot | Robot learning state |
| Gray faded | Previous iterations |

## Troubleshooting

### Cannot connect to simulator
```bash
# Check if simulator is running
ps aux | grep ilc_simulator

# Check port
netstat -an | grep 8765

# Restart simulator
pkill ilc_simulator
./build/ilc_simulator
```

### X11 display error
```bash
# Set DISPLAY variable
export DISPLAY=:0

# For WSL2, install X server (VcXsrv)
```

### Build errors
```bash
# Clean and rebuild
make clean && make

# Check dependencies
dpkg -l | grep libx11-dev
```

## Scripting Examples

### Automated Test
```bash
#!/bin/bash
(
    echo "start"
    sleep 2
    echo "error 0.3"
    sleep 10
    echo "status"
    echo "stop"
) | ./build/ilc_client
```

### Parameter Sweep
```bash
#!/bin/bash
for lr in 0.2 0.4 0.6 0.8; do
    echo "Testing LR=$lr"
    (
        echo "reset"
        echo "start"
        echo "lr $lr"
        echo "error 0.3"
        sleep 15
        echo "status"
    ) | ./build/ilc_client
done
```

### Convergence Monitor
```bash
#!/bin/bash
while true; do
    clear
    echo "status" | ./build/ilc_client | grep -E "(Iteration|RMS)"
    sleep 2
done
```

## Algorithm Tuning

### Fast Convergence (5-15 iterations)
```bash
ilc> lr 0.7
ilc> smooth 0.8
# Effective rate: 0.56
```

### Smooth Convergence (20-40 iterations)
```bash
ilc> lr 0.5
ilc> smooth 0.3
# Effective rate: 0.15
```

### Balanced (10-25 iterations) ⭐ Recommended
```bash
ilc> lr 0.6
ilc> smooth 0.6
# Effective rate: 0.36
```

### Handling Noise
```bash
ilc> lr 0.4        # Lower LR for noise
ilc> smooth 0.5    # Moderate smoothing
ilc> noise on
```

## Performance Tips

- **Smooth paths**: Use α=0.2-0.4
- **Fast learning**: Use L=0.6-0.8, α=0.5-0.7
- **Noisy systems**: Use L=0.2-0.4, α=0.1-0.3
- **Large errors**: Start with L=0.3, α=0.3, then increase

## Keyboard Shortcuts in Client

| Key | Action |
|-----|--------|
| `Ctrl+C` | Exit client |
| `Ctrl+D` | Exit client |
| `Up/Down` | Command history (if terminal supports) |

## Common Issues

### Path not converging
- Lower learning rate: `lr 0.3`
- Increase smoothing: `smooth 0.2`
- Check error level: `status`

### Oscillating/zig-zagging
- Reduce learning rate: `lr 0.4`
- Reduce smoothing: `smooth 0.3`
- May indicate instability

### Slow convergence
- Increase learning rate: `lr 0.6`
- Increase smoothing: `smooth 0.5`
- Check if error level is too high

## File Structure

```
ilc_tracker/
├── simulator.cpp       # Main ILC engine + visualization
├── client.cpp          # Command interface
├── Makefile           # Build system
├── build.sh           # Build script
├── demo.sh            # Automated demo
├── setup.sh           # Installation script
├── README.md          # Full documentation
├── QUICKREF.md        # This file
└── build/
    ├── ilc_simulator  # Compiled simulator
    └── ilc_client     # Compiled client
```

## Getting Help

```bash
# In client
ilc> help

# Show Makefile options
make help

# Read full docs
cat README.md | less
```

## Quick Demo

```bash
# Run automated demo
chmod +x demo.sh
./demo.sh

# Or use make
make demo
```

## One-Line Tests

```bash
# Basic convergence test
echo -e "start\nerror 0.3\nstatus" | ./build/ilc_client

# Shape test
echo -e "start\nshape star\nerror 0.4" | ./build/ilc_client

# Preset test
echo -e "start\npreset noise\nstatus" | ./build/ilc_client
```

## Cheat Sheet: Typical Session

```
1. Start simulator:        make run-sim
2. Start client:           make run-client
3. Begin motion:           start
4. Add disturbance:        error 0.3
5. Watch ILC learn:        (wait 10-20 seconds)
6. Check progress:         status
7. Try different shape:    shape ellipse
8. Adjust parameters:      lr 0.5, smooth 0.4
9. Test preset:            preset noise
10. Stop and reset:        stop, reset
```

---

**Pro Tip**: Keep this file open in a separate terminal while experimenting!

```bash
# View in terminal
less QUICKREF.md

# Or in browser
xdg-open QUICKREF.md  # Linux
open QUICKREF.md      # macOS
```
