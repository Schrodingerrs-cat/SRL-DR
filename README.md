```markdown
# ILC Real-Time Tracker  
### Iterative Learning Control with Spherical Dome Construction and STL Export

---

## 1. What This Project Actually Is

This codebase implements a **closed-loop Iterative Learning Control (ILC) system** that:

1. Tracks a **closed 2D reference path**
2. Learns from tracking error **across iterations**
3. Stores each converged trajectory as a **new physical layer**
4. Converts those layers into a **3D object**
5. Optionally **shrinks the reference on a sphere** to form a **dome**
6. Exports the result as a **3D-printable STL (hollow shell)**

This is **not** just visualization.  
It is a **learning system that constructs geometry from control convergence**.

---

## 2. Mental Model (Very Important)

Think of the system as a **robot tracing the same loop repeatedly**:

- Iteration 0: bad tracking  
- Iteration 1: slightly better  
- Iteration k: converged  

Each iteration becomes **one physical layer in Z**.

So:

> **Learning over time → geometry over height**

---

## 3. High-Level Loop (End-to-End)

Every frame:

1. A point index advances along the reference path
2. A command is generated using learned corrections
3. A simulated plant distorts the command
4. Error is accumulated
5. After a full loop:
   - RMS + mean error are computed
   - Corrections are updated
   - The trajectory is frozen as a completed layer
6. If dome mode is active:
   - The reference may shrink
   - A new Z height is computed from sphere geometry

---

## 4. Iterative Learning Control (ILC) — Core Theory

### 4.1 What ILC Solves

ILC assumes:
- The task is **repeated**
- The disturbance is **repeatable**
- We can learn from **previous executions**

Unlike PID:
- PID reacts **within** an iteration
- ILC learns **across** iterations

---

### 4.2 State Variables

For iteration `k` and path index `i`:

- Reference:  
  `r_i`
- Commanded position:  
  `u_k(i)`
- Actual position:  
  `y_k(i)`
- Error:  
  `e_k(i) = r_i − y_k(i)`
- Correction term:  
  `Δu_k(i)`

---

### 4.3 Learning Law (Implemented Exactly)

```

u_{k+1}(i) = u_k(i) + α · L(e_k(i))

````

Where:
- `α` = learning rate
- `L` = smoothing + clamping operator

In code:
```python
d_x = learning_rate * error.x
d_y = learning_rate * error.y
````

Then:

* Smoothed via `smoothing_alpha`
* Clamped by `MAX_DELTA_PER_ITER`
* Accumulated or replaced depending on mode

---

### 4.4 Why Smoothing Exists

Without smoothing:

* Corrections oscillate
* High-frequency noise explodes

With smoothing:

```python
eff_dx = smoothing_alpha * d_x
```

This acts as a **low-pass filter in iteration space**.

---

### 4.5 Why Clamping Exists

Two clamps are enforced:

1. **Per-iteration update**

```python
MAX_DELTA_PER_ITER
```

2. **Total correction magnitude**

```python
MAX_CORRECTION_MAG
```

These prevent:

* Divergence
* Fold-over geometry
* Self-intersections

---

## 5. Plant Model (Why Errors Exist)

The plant is **intentionally nonlinear**.

It applies:

* Radial distortion
* Phase shift
* Contraction
* Optional noise

This simulates:

* Calibration error
* Compliance
* Systematic bias

ILC’s job is to **cancel this repeatable distortion**.

---

## 6. When an Iteration Is “Complete”

An iteration completes when:

* The full closed loop has been traversed
* All `NUM_POINTS` have been visited

At that moment:

* RMS error is computed
* Mean absolute error is computed
* Corrections are updated
* The trajectory is **frozen as a layer**

This is critical:

> **A layer is not a time step — it is a converged execution**

---

## 7. Z-Stacking: How 2D Becomes 3D

### 7.1 Cylinder Phase (Normal Stacking)

Before dome mode:

```python
z = (iteration_index + 1) * LAYER_DZ_WORLD
```

So each iteration becomes a flat slice.

This forms a **straight-walled cylinder**.

---

## 8. Dome Mode — Conceptual Explanation

### 8.1 What Dome Mode Means

Instead of stacking identical circles:

* The reference **shrinks**
* The shrink follows a **sphere**
* Z is computed from sphere geometry

So we are effectively tracing **horizontal slices of a sphere**.

---

### 8.2 Sphere Geometry Used

Let:

* `R` = sphere radius (mm)
* `r` = current reference radius (mm)

Then:

```
z = sqrt(R² − r²)
```

Converted to world units.

This ensures:

* Large dz at base
* dz → 0 near top
* Smooth spherical cap

---

### 8.3 Why Z “Slows Down” Near the Top

Mathematically:

```
d/d r sqrt(R² − r²) → 0 as r → 0
```

This is **correct spherical behavior**, not a bug.

---

## 9. Dome Adaptation Logic (Critical)

### 9.1 Shrink Happens Only After Convergence

If:

```python
mean_error >= 0.01
```

Then:

* No shrink
* Only another layer is added

This guarantees:

* Each radius level is learned before shrinking
* The dome is stable

---

### 9.2 Adaptive Learning Rate During Dome

Learning rate is **reduced as geometry shrinks**:

* Large radius → higher LR
* Small radius → lower LR

This avoids:

* Instability near the apex
* Overshoot on tiny circles

---

## 10. Minimum Printable Geometry Constraint

Defined by:

```python
MIN_DIAMETER_MM
MIN_AREA_WORLD
```

Once the area drops below this:

* Dome is marked complete
* Simulation auto-stops

This enforces **physical printability**, not math convenience.

---

## 11. STL Export — What Geometry Is Generated

### 11.1 STL Mode Used (C2)

This exporter generates:

* **Side walls only**
* Continuous vertical walls
* No caps
* Real thickness via inner + outer walls

This is equivalent to **vase mode** in slicers.

---

### 11.2 Why Caps Are Omitted

Caps:

* Break vase printing
* Create non-manifold edges
* Are printer-dependent

This code intentionally exports **structural walls only**.

---

## 12. Runtime-Configurable Parameters (Client)

These can be changed **without restarting**.

| Parameter     | Command  | Meaning                     |
| ------------- | -------- | --------------------------- |
| Learning rate | `lr`     | Speed of learning           |
| Smoothing     | `smooth` | Stability vs responsiveness |
| Error level   | `error`  | Plant distortion            |
| Noise         | `noise`  | Stochastic disturbance      |
| Shape         | `shape`  | Reference geometry          |
| Dome          | `dome`   | Enable spherical shrink     |
| 3D dz         | `plot3d` | Visualization spacing       |

---

## 13. Fixed Parameters (Code Only)

These define **system identity**.

### Geometry

```python
NUM_POINTS
WORLD_MIN / WORLD_MAX
```

### Learning Safety

```python
MAX_DELTA_PER_ITER
MAX_CORRECTION_MAG
```

### Physical Meaning

```python
MM_PER_WORLD
MIN_DIAMETER_MM
```

### Layering

```python
LAYER_DZ_WORLD
```

Changing these **changes the physics of the system**.

---

## 14. Typical Experiment Flow

1. Start simulation
2. Inject error
3. Observe convergence
4. Enable dome
5. Let geometry shrink
6. Auto-stop
7. Export STL

---

## 15. What This System Is Good For

* Learning-based additive manufacturing
* Toolpath learning
* Geometry synthesis from control
* Research demos
* Educational ILC visualization

---

## 16. One-Sentence Summary

> This system **learns how to draw a shape**, then **turns that learning process into a physical 3D object**, with geometry governed by **control convergence and spherical constraints**, not by pre-defined CAD.
