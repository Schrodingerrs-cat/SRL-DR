/**
     * Complete one full iteration and update ILC corrections
     * 
     * This is the heart of the ILC algorithm. Called after the robot completes
     * one full trajectory cycle. It:
     *   1. Calculates tracking error at each point
     *   2. Computes RMS error for convergence monitoring
     *   3. Updates corrections using the ILC update law with smoothing
     *   4. Stores the completed trajectory for visualization
     * 
     * Standard ILC Update Law:
     *   e_k(t) = reference(t) - actual_k(t)      // Calculate error
     *   u_{k+1}(t) = u_k(t) + α * L * e_k(t)    // Update correction
     * 
     * The smoothing factor α prevents oscillations:
     *   - α near 0.1: Very gradual updates, smooth but slow (40+ iterations)
     *   - α near 0.5: Balanced convergence (15-25 iterations)
     *   - α near 1.0: Aggressive updates, fast but may oscillate (8-15 iterations)
     * 
     * Effective learning rate = α × L
     * For fast convergence: Use L=0.7, α=0.8 → effective rate = 0./*******************************************************************************
 * ILC Real-Time Tracker - Main Simulator
 * 
 * Author: [Your Name]
 * Date: October 2025
 * Version: 1.0
 * 
 * Description:
 *   This program implements a real-time Iterative Learning Control (ILC) 
 *   system for trajectory tracking. The robot learns to follow reference 
 *   trajectories (circle, ellipse, square, star) by iteratively correcting 
 *   systematic errors over multiple trials.
 * 
 * Key Features:
 *   - Standard ILC algorithm with smoothing factor
 *   - Real-time X11 visualization
 *   - TCP command server for remote control
 *   - Multiple trajectory shapes with on-the-fly switching
 *   - Configurable systematic errors and noise
 *   - Smooth correction updates to prevent oscillation
 * 
 * ILC Theory:
 *   The algorithm uses the classic ILC update law:
 *     u_{k+1}(t) = u_k(t) + α * L * e_k(t)
 *   where:
 *     - u_k(t) is the feedforward correction at time t, iteration k
 *     - e_k(t) = reference(t) - actual_k(t) is the tracking error
 *     - L is the learning rate (0.1-0.8)
 *     - α is the smoothing factor (0.1-1.0)
 *     - Effective learning rate = α × L
 * 
 * Usage:
 *   Terminal 1: ./ilc_simulator
 *   Terminal 2: ./ilc_client (for commands)
 * 
 * License: MIT
 ******************************************************************************/

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================

// Window and visualization settings
const int WINDOW_SIZE = 800;        // X11 window size in pixels
const int MARGIN = 60;              // Border margin around plot area
const double WORLD_MIN = -1.8;      // Minimum world coordinate
const double WORLD_MAX = 1.8;       // Maximum world coordinate

// ILC algorithm parameters
const int NUM_POINTS = 150;         // Number of points in trajectory
const int SERVER_PORT = 8765;       // TCP server port for commands

// ============================================================================
// MATH UTILITIES
// ============================================================================

/**
 * Point2D - Simple 2D point structure
 * 
 * Represents a point in 2D Cartesian coordinates. Used for both reference
 * trajectories and actual robot positions.
 */
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

/**
 * Calculate Euclidean distance between two points
 * 
 * @param a First point
 * @param b Second point
 * @return Distance between points
 */
inline double distance(const Point2D& a, const Point2D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

/**
 * Convert world coordinates to screen coordinates
 * 
 * Maps from world space [-1.8, 1.8] to screen space [MARGIN, WINDOW_SIZE-MARGIN]
 * 
 * @param val World coordinate value
 * @return Corresponding screen pixel coordinate
 */
inline int worldToScreen(double val) {
    return static_cast<int>(MARGIN + (val - WORLD_MIN) / (WORLD_MAX - WORLD_MIN) * (WINDOW_SIZE - 2*MARGIN));
}

// ============================================================================
// SHAPE GENERATOR
// ============================================================================

/**
 * ShapeGenerator - Factory class for creating reference trajectories
 * 
 * Generates various closed-path reference trajectories for the robot to track.
 * All shapes are parameterized and can be scaled/adjusted.
 */
class ShapeGenerator {
public:
    /**
     * Generate a circular reference trajectory
     * 
     * Creates evenly-spaced points around a circle using parametric equations:
     *   x(θ) = r * cos(θ)
     *   y(θ) = r * sin(θ)
     * 
     * @param numPoints Number of waypoints around circle
     * @param radius Circle radius in world units
     * @return Vector of points forming a circle
     */
    static std::vector<Point2D> generateCircle(int numPoints, double radius = 1.0) {
        std::vector<Point2D> points;
        for (int i = 0; i < numPoints; i++) {
            double theta = (2.0 * M_PI * i) / numPoints;  // Angle for each point
            points.push_back(Point2D(radius * std::cos(theta), radius * std::sin(theta)));
        }
        return points;
    }
    
    /**
     * Generate an elliptical reference trajectory
     * 
     * Creates an ellipse using parametric equations:
     *   x(θ) = a * cos(θ)  (semi-major axis)
     *   y(θ) = b * sin(θ)  (semi-minor axis)
     * 
     * @param numPoints Number of waypoints around ellipse
     * @param a Semi-major axis length (horizontal stretch)
     * @param b Semi-minor axis length (vertical stretch)
     * @return Vector of points forming an ellipse
     */
    static std::vector<Point2D> generateEllipse(int numPoints, double a = 1.2, double b = 0.7) {
        std::vector<Point2D> points;
        for (int i = 0; i < numPoints; i++) {
            double theta = (2.0 * M_PI * i) / numPoints;
            points.push_back(Point2D(a * std::cos(theta), b * std::sin(theta)));
        }
        return points;
    }
    
    /**
     * Generate a square reference trajectory
     * 
     * Creates a square path by dividing points equally among four sides.
     * Points are linearly interpolated along each edge.
     * 
     * @param numPoints Number of waypoints (distributed across 4 sides)
     * @param side Length of each side of the square
     * @return Vector of points forming a square
     */
    static std::vector<Point2D> generateSquare(int numPoints, double side = 1.6) {
        std::vector<Point2D> points;
        int pointsPerSide = numPoints / 4;  // Divide points among 4 sides
        double half = side / 2.0;           // Half-side length
        
        // Bottom edge: left to right
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(-half + side * t, -half));
        }
        
        // Right edge: bottom to top
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(half, -half + side * t));
        }
        
        // Top edge: right to left
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(half - side * t, half));
        }
        
        // Left edge: top to bottom
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(-half, half - side * t));
        }
        
        // Pad with last point if needed to reach exact numPoints
        while (points.size() < (size_t)numPoints) {
            points.push_back(points.back());
        }
        points.resize(numPoints);
        return points;
    }
    
    /**
     * Generate a 5-pointed star reference trajectory
     * 
     * Creates a star by alternating between outer and inner radius vertices.
     * Uses 5 peaks with interpolated points between them.
     * 
     * @param numPoints Number of waypoints around star
     * @param outerRadius Radius to outer points (peaks)
     * @param innerRadius Radius to inner points (valleys)
     * @return Vector of points forming a star
     */
    static std::vector<Point2D> generateStar(int numPoints, double outerRadius = 1.0, double innerRadius = 0.4) {
        std::vector<Point2D> points;
        int peaks = 5;                          // 5-pointed star
        int pointsPerPeak = numPoints / (peaks * 2);  // Points per edge
        
        for (int i = 0; i < peaks * 2; i++) {
            double angle = (M_PI * i) / peaks;  // Current vertex angle
            double radius = (i % 2 == 0) ? outerRadius : innerRadius;  // Alternate radius
            
            // Interpolate points along edge to next vertex
            for (int j = 0; j < pointsPerPeak; j++) {
                double t = (double)j / pointsPerPeak;  // Interpolation parameter
                double nextAngle = (M_PI * (i + 1)) / peaks;
                double nextRadius = ((i + 1) % 2 == 0) ? outerRadius : innerRadius;
                
                // Linear interpolation in polar coordinates
                double r = radius + t * (nextRadius - radius);
                double a = angle + t * (nextAngle - angle);
                points.push_back(Point2D(r * std::cos(a), r * std::sin(a)));
            }
        }
        
        points.resize(numPoints);
        return points;
    }
};

// ============================================================================
// ILC CONTROLLER
// ============================================================================

/**
 * ILCController - Main Iterative Learning Control algorithm implementation
 * 
 * This class implements the standard ILC algorithm for trajectory tracking.
 * ILC works by learning feedforward corrections over repeated trials, making
 * it ideal for repetitive tasks where systematic errors occur.
 * 
 * Algorithm:
 *   1. Execute trajectory and measure error: e_k(t) = reference(t) - actual_k(t)
 *   2. Update correction: u_{k+1}(t) = u_k(t) + α * L * e_k(t)
 *   3. Repeat until error converges to zero
 * 
 * The smoothing factor α prevents abrupt changes in the correction signal,
 * which can cause oscillations. Lower α = smoother but slower convergence.
 * 
 * Plant Model:
 *   Simulates systematic errors (phase lag, radial distortion, contraction)
 *   and optional random noise to test ILC robustness.
 */
class ILCController {
private:
    // Core parameters
    int numPoints;              // Number of discrete points in trajectory
    double learningRate;        // L: How much to trust the error signal (0.1-0.8)
    double systemErrorLevel;    // Magnitude of induced systematic errors (0-1)
    int iteration;              // Current iteration number (trial count)
    bool enableNoise;           // Whether to add random disturbances
    double smoothingAlpha;      // α: Smoothing factor for gradual updates (0.1-1.0)
    
    // Trajectory data
    std::vector<Point2D> reference;           // Desired trajectory (fixed)
    std::vector<Point2D> corrections;         // ILC feedforward corrections u_k
    std::vector<Point2D> currentTrajectory;   // Robot's actual path this iteration
    std::vector<Point2D> lastErrors;          // Tracking errors from last iteration
    std::vector<std::vector<Point2D>> completedPaths;  // History of past trajectories
    
    // Performance metrics
    double lastRMSError;        // Root-mean-square tracking error
    
public:
    /**
     * Constructor - Initialize ILC controller
     * 
     * @param nPts Number of points in trajectory (typically 150)
     * @param lr Initial learning rate (recommended: 0.4-0.6)
     */
    ILCController(int nPts, double lr) 
        : numPoints(nPts), learningRate(lr), systemErrorLevel(0.0), 
          iteration(0), enableNoise(false), smoothingAlpha(0.3), lastRMSError(0.0) {
        corrections.resize(numPoints, Point2D(0, 0));  // Initialize corrections to zero
        reference = ShapeGenerator::generateCircle(numPoints);  // Default: circle
    }
    
    /**
     * Set new reference trajectory
     * 
     * Changes the desired path that the robot should follow.
     * This allows switching between different shapes (circle, ellipse, etc.)
     * 
     * @param ref New reference trajectory
     */
    void setReference(const std::vector<Point2D>& ref) {
        reference = ref;
        reference.resize(numPoints);
    }
    
    /**
     * Plant model - Simulates robot dynamics with systematic errors
     * 
     * This function represents the physical system (robot + disturbances).
     * It takes a commanded position and returns the actual achieved position,
     * which may differ due to systematic errors.
     * 
     * Error Model:
     *   - Radial distortion: Multi-harmonic deformation (3rd, 5th harmonics)
     *   - Phase lag: Timing/synchronization errors that vary with iteration
     *   - Contraction: Non-uniform scaling that varies around the path
     *   - Random noise: Stochastic disturbances (optional)
     * 
     * These errors are repeatable (systematic) except for noise, which means
     * ILC can learn to compensate for them over multiple iterations.
     * 
     * @param command Desired position command from controller
     * @param pathIndex Current point index in trajectory (0 to NUM_POINTS-1)
     * @return Actual achieved position (with errors applied)
     */
    Point2D plantModel(const Point2D& command, int pathIndex) {
        Point2D output = command;  // Start with commanded position
        
        // Only apply errors if systemErrorLevel > 0
        if (systemErrorLevel > 0) {
            double theta = (2.0 * M_PI * pathIndex) / numPoints;  // Angular position
            
            // Multi-harmonic radial distortion
            // Uses superposition of multiple frequency components
            double radialError = systemErrorLevel * (
                0.25 * std::sin(3 * theta) +   // 3rd harmonic (3 bumps around circle)
                0.15 * std::sin(5 * theta) +   // 5th harmonic (5 bumps)
                0.10 * std::cos(2 * theta)     // 2nd harmonic (2 bumps, phase-shifted)
            );
            
            // Phase lag error - timing offset that varies slightly with iteration
            // This simulates phase desynchronization that drifts slowly
            double phaseError = systemErrorLevel * (0.18 + 0.02 * std::sin(0.5 * theta + iteration));
            
            // Contraction error - non-uniform scaling around the path
            // Causes the shape to deform (squeeze/stretch)
            double contractionError = systemErrorLevel * (0.12 + 0.03 * std::cos(1.5 * theta));
            
            // Convert to polar coordinates for applying errors
            double radius = std::sqrt(output.x * output.x + output.y * output.y);
            double angle = std::atan2(output.y, output.x);
            
            // Apply errors in polar form
            double newRadius = radius * (1 - contractionError) + radialError;
            double newAngle = angle + phaseError;
            
            // Convert back to Cartesian coordinates
            output.x = newRadius * std::cos(newAngle);
            output.y = newRadius * std::sin(newAngle);
            
            // Add optional random noise (non-repeatable disturbance)
            // ILC can't learn to compensate for this, but should be robust to it
            if (enableNoise) {
                double noiseAmp = systemErrorLevel * 0.04;  // Noise amplitude
                output.x += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
                output.y += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
            }
        }
        
        return output;
    }
    
    /**
     * Get current robot position with ILC correction applied
     * 
     * This is called at each time step during trajectory execution.
     * It applies the learned ILC correction to the reference, sends the
     * corrected command through the plant model, and records the result.
     * 
     * Control Law:
     *   command(t) = reference(t) + correction(t)
     *   actual(t) = plantModel(command(t))
     * 
     * @param pathIndex Current point index in trajectory
     * @return Actual robot position at this time step
     */
    Point2D getCurrentPosition(int pathIndex) {
        if (pathIndex >= numPoints) pathIndex = numPoints - 1;  // Bounds check
        
        Point2D refPoint = reference[pathIndex];        // Desired position
        Point2D correction = corrections[pathIndex];    // Learned ILC correction
        
        // Apply full correction (no damping here - smoothing is in the update law)
        // This is the feedforward command sent to the plant
        Point2D command;
        command.x = refPoint.x + correction.x;
        command.y = refPoint.y + correction.y;
        
        // Get actual position from plant (with errors)
        Point2D actual = plantModel(command, pathIndex);
        
        // Record actual position for error calculation at end of iteration
        if ((int)currentTrajectory.size() <= pathIndex) {
            currentTrajectory.push_back(actual);
        } else {
            currentTrajectory[pathIndex] = actual;
        }
        
        return actual;
    }
    
    /**
     * Complete one full iteration and update ILC corrections
     * 
     * This is the heart of the ILC algorithm. Called after the robot completes
     * one full trajectory cycle. It:
     *   1. Calculates tracking error at each point
     *   2. Computes RMS error for convergence monitoring
     *   3. Updates corrections using the ILC update law with smoothing
     *   4. Stores the completed trajectory for visualization
     * 
     * Standard ILC Update Law:
     *   e_k(t) = reference(t) - actual_k(t)      // Calculate error
     *   u_{k+1}(t) = u_k(t) + α * L * e_k(t)    // Update correction
     * 
     * The smoothing factor α prevents oscillations:
     *   - α near 0.1: Very gradual updates, smooth but slow (40+ iterations)
     *   - α near 0.5: Balanced convergence (15-25 iterations)
     *   - α near 1.0: Aggressive updates, fast but may oscillate (8-15 iterations)
     * 
     * Effective learning rate = α × L
     * For fast convergence: Use L=0.7, α=0.8 → effective rate = 0.56
     * 
     * @return RMS tracking error for this iteration
     */
    double completeIteration() {
        // Safety check - need complete trajectory before updating
        if (currentTrajectory.size() < (size_t)numPoints) return lastRMSError;
        
        std::cout << "[ILC] Completing iteration " << iteration << std::endl;
        
        std::vector<Point2D> trackingErrors;
        double totalError = 0;
        
        // STEP 1: Calculate tracking errors
        // Standard ILC: Error is always measured against the FIXED reference,
        // not against the previous iteration. This is key!
        for (int i = 0; i < numPoints; i++) {
            Point2D refPt = reference[i];          // Desired position (fixed)
            Point2D actualPt = currentTrajectory[i];  // Actual achieved position
            
            // Direct error calculation: e_k(i) = reference(i) - actual_k(i)
            // This is the standard ILC formulation
            Point2D error;
            error.x = refPt.x - actualPt.x;
            error.y = refPt.y - actualPt.y;
            trackingErrors.push_back(error);
            
            // Accumulate squared error for RMS calculation
            totalError += error.x * error.x + error.y * error.y;
        }
        
        // STEP 2: Calculate RMS error (performance metric)
        // RMS = sqrt(mean(squared errors))
        // This gives us a single number to track convergence
        double rmsError = std::sqrt(totalError / numPoints);
        lastRMSError = rmsError;
        
        // STEP 3: Update ILC corrections using smoothed update law
        // u_{k+1}(i) = u_k(i) + α * L * e_k(i)
        // where e_k(i) = reference(i) - actual_k(i)
        // 
        // Smoothing approach: Instead of immediately jumping to the new correction,
        // we blend it gradually with the old correction. This prevents abrupt
        // changes that can cause the robot to overshoot or oscillate.
        for (int i = 0; i < numPoints && i < (int)trackingErrors.size(); i++) {
            // Calculate how much we WOULD change the correction (without smoothing)
            double proposedDeltaX = learningRate * trackingErrors[i].x;
            double proposedDeltaY = learningRate * trackingErrors[i].y;
            
            // Apply smoothing factor α:
            // - Low α (0.1-0.3): Accept only a small fraction of the proposed change
            //   Result: Very smooth path, but takes many iterations to converge
            // - High α (0.7-1.0): Accept most/all of the proposed change
            //   Result: Fast convergence, but may oscillate
            // 
            // Think of α as a "trust factor" - how much do we trust this error signal?
            corrections[i].x += smoothingAlpha * proposedDeltaX;
            corrections[i].y += smoothingAlpha * proposedDeltaY;
            
            // STEP 4: Limit correction magnitude (safety/stability)
            // Prevent corrections from growing unboundedly, which could indicate
            // instability or cause the robot to command positions far outside
            // the workspace. This is a practical safeguard.
            double correctionMag = std::sqrt(corrections[i].x * corrections[i].x + 
                                            corrections[i].y * corrections[i].y);
            if (correctionMag > 1.0) {
                // Normalize to maximum allowed magnitude
                corrections[i].x = (corrections[i].x / correctionMag) * 1.0;
                corrections[i].y = (corrections[i].y / correctionMag) * 1.0;
            }
        }
        
        // STEP 5: Bookkeeping and cleanup
        iteration++;  // Increment iteration counter
        lastErrors = trackingErrors;  // Save errors for potential visualization
        
        std::cout << "[ILC] Iteration " << iteration-1 << " RMS Error: " << rmsError << std::endl;
        
        // Store completed trajectory for visualization (path history)
        completedPaths.push_back(currentTrajectory);
        if (completedPaths.size() > 5) {
            completedPaths.erase(completedPaths.begin());  // Keep only last 5
        }
        
        // Clear current trajectory buffer for next iteration
        currentTrajectory.clear();
        
        return rmsError;
    }
    
    /**
     * Induce systematic error in the plant
     * 
     * Simulates real-world disturbances like calibration errors, friction,
     * backlash, thermal drift, etc. These are repeatable errors that ILC
     * can learn to compensate for.
     * 
     * @param level Error magnitude (0.0 = perfect, 1.0 = severe distortion)
     */
    void induceError(double level) {
        systemErrorLevel = level;
        std::cout << "[ILC] System error set to " << (level * 100) << "%" << std::endl;
    }
    
    /**
     * Set ILC learning rate
     * 
     * Controls how much to trust the error signal. Higher values lead to
     * faster convergence but may cause oscillations if too high.
     * 
     * Typical range: 0.1-0.8
     * Recommended: 0.4-0.6 for stable learning
     * 
     * @param lr New learning rate
     */
    void setLearningRate(double lr) {
        learningRate = lr;
        std::cout << "[ILC] Learning rate set to " << lr << std::endl;
    }
    
    /**
     * Set smoothing factor for ILC updates
     * 
     * Controls how aggressively corrections are updated each iteration.
     * This is the α parameter in: u_{k+1} = u_k + α * L * e_k
     * 
     * Lower values (0.1-0.3):
     *   + Very smooth correction trajectory
     *   + No oscillations
     *   - Slow convergence (30-50 iterations)
     * 
     * Higher values (0.7-1.0):
     *   + Fast convergence (8-15 iterations)
     *   - May oscillate if learning rate also high
     * 
     * Balanced (0.4-0.6):
     *   Moderate speed, good stability
     * 
     * @param alpha Smoothing factor (clamped to 0.1-1.0)
     */
    void setSmoothingFactor(double alpha) {
        smoothingAlpha = std::max(0.1, std::min(1.0, alpha));
        std::cout << "[ILC] Smoothing factor set to " << smoothingAlpha << std::endl;
        std::cout << "      (Controls rate of correction update - higher = faster learning)" << std::endl;
    }
    
    /**
     * Enable or disable random noise in plant
     * 
     * Adds stochastic disturbances to test ILC robustness. Unlike systematic
     * errors, random noise is non-repeatable, so ILC cannot learn to compensate.
     * However, ILC should still converge to the repeatable component.
     * 
     * @param enabled True to enable noise, false to disable
     */
    void setNoise(bool enabled) {
        enableNoise = enabled;
        std::cout << "[ILC] Noise " << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    /**
     * Reset ILC - Clear all learning
     * 
     * Resets the controller to initial state:
     *   - Clears all learned corrections (u = 0)
     *   - Resets iteration counter
     *   - Clears trajectory history
     *   - Removes systematic errors
     * 
     * Use this when switching reference trajectories or testing convergence
     * from scratch.
     */
    void reset() {
        corrections.assign(numPoints, Point2D(0, 0));  // Zero all corrections
        currentTrajectory.clear();
        lastErrors.clear();
        completedPaths.clear();
        iteration = 0;
        systemErrorLevel = 0;
        lastRMSError = 0;
        std::cout << "[ILC] Reset complete" << std::endl;
    }
    
    // ========================================================================
    // Getters - Provide read-only access to internal state
    // ========================================================================
    
    const std::vector<Point2D>& getReference() const { return reference; }
    const std::vector<Point2D>& getCurrentTrajectory() const { return currentTrajectory; }
    const std::vector<std::vector<Point2D>>& getCompletedPaths() const { return completedPaths; }
    const std::vector<Point2D>& getLastErrors() const { return lastErrors; }
    int getIteration() const { return iteration; }
    double getRMSError() const { return lastRMSError; }
    double getErrorLevel() const { return systemErrorLevel; }
    bool getNoiseEnabled() const { return enableNoise; }
    
    /**
     * Calculate average correction magnitude
     * 
     * Provides a metric for how much ILC is correcting. As learning progresses,
     * this value stabilizes (corrections stop changing significantly).
     * 
     * @return Average magnitude of ILC corrections
     */
    double getAvgCorrection() const {
        double sum = 0;
        for (const auto& c : corrections) {
            sum += std::sqrt(c.x*c.x + c.y*c.y);
        }
        return sum / numPoints;
    }
};

// ============================================================================
// X11 VISUALIZER
// ============================================================================
class X11Visualizer {
private:
    Display* display;
    Window window;
    GC gc;
    int screen;
    XColor colors[10];
    
public:
    X11Visualizer() {
        display = XOpenDisplay(nullptr);
        if (!display) {
            std::cerr << "Cannot open X display" << std::endl;
            exit(1);
        }
        
        screen = DefaultScreen(display);
        window = XCreateSimpleWindow(display, RootWindow(display, screen),
                                     10, 10, WINDOW_SIZE, WINDOW_SIZE, 1,
                                     BlackPixel(display, screen),
                                     WhitePixel(display, screen));
        
        XSelectInput(display, window, ExposureMask | KeyPressMask);
        XMapWindow(display, window);
        XStoreName(display, window, "ILC Real-Time Tracker");
        
        gc = XCreateGC(display, window, 0, nullptr);
        
        // Initialize colors
        Colormap colormap = DefaultColormap(display, screen);
        XParseColor(display, colormap, "#22c55e", &colors[0]); // Green (reference)
        XParseColor(display, colormap, "#2563eb", &colors[1]); // Blue (trajectory)
        XParseColor(display, colormap, "#dc2626", &colors[2]); // Red (error)
        XParseColor(display, colormap, "#f59e0b", &colors[3]); // Orange (robot learning)
        XParseColor(display, colormap, "#f0f0f0", &colors[4]); // Grid
        XParseColor(display, colormap, "#666666", &colors[5]); // Dark gray
        
        for (int i = 0; i < 6; i++) {
            XAllocColor(display, colormap, &colors[i]);
        }
    }
    
    ~X11Visualizer() {
        XFreeGC(display, gc);
        XDestroyWindow(display, window);
        XCloseDisplay(display);
    }
    
    void clear() {
        XClearWindow(display, window);
    }
    
    void drawGrid() {
        XSetForeground(display, gc, colors[4].pixel);
        XSetLineAttributes(display, gc, 1, LineSolid, CapButt, JoinMiter);
        
        double gridLines[] = {-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5};
        for (double val : gridLines) {
            int pos = worldToScreen(val);
            XDrawLine(display, window, gc, pos, MARGIN, pos, WINDOW_SIZE - MARGIN);
            XDrawLine(display, window, gc, MARGIN, pos, WINDOW_SIZE - MARGIN, pos);
        }
    }
    
    void drawPath(const std::vector<Point2D>& path, int colorIdx, int lineWidth, bool closed = true) {
        if (path.size() < 2) return;
        
        XSetForeground(display, gc, colors[colorIdx].pixel);
        XSetLineAttributes(display, gc, lineWidth, LineSolid, CapRound, JoinRound);
        
        for (size_t i = 0; i < path.size() - 1; i++) {
            int x1 = worldToScreen(path[i].x);
            int y1 = worldToScreen(path[i].y);
            int x2 = worldToScreen(path[i+1].x);
            int y2 = worldToScreen(path[i+1].y);
            XDrawLine(display, window, gc, x1, y1, x2, y2);
        }
        
        if (closed && path.size() > 2) {
            int x1 = worldToScreen(path.back().x);
            int y1 = worldToScreen(path.back().y);
            int x2 = worldToScreen(path[0].x);
            int y2 = worldToScreen(path[0].y);
            XDrawLine(display, window, gc, x1, y1, x2, y2);
        }
    }
    
    void drawRobot(const Point2D& pos, int colorIdx, int radius = 10) {
        int x = worldToScreen(pos.x);
        int y = worldToScreen(pos.y);
        
        XSetForeground(display, gc, colors[colorIdx].pixel);
        XFillArc(display, window, gc, x - radius, y - radius, 
                radius * 2, radius * 2, 0, 360 * 64);
        
        // White border
        XSetForeground(display, gc, WhitePixel(display, screen));
        XSetLineAttributes(display, gc, 2, LineSolid, CapButt, JoinMiter);
        XDrawArc(display, window, gc, x - radius, y - radius,
                radius * 2, radius * 2, 0, 360 * 64);
    }
    
    void drawText(int x, int y, const std::string& text) {
        XSetForeground(display, gc, BlackPixel(display, screen));
        XDrawString(display, window, gc, x, y, text.c_str(), text.length());
    }
    
    void flush() {
        XFlush(display);
    }
};

// ============================================================================
// COMMAND SERVER (TCP Socket)
// ============================================================================
class CommandServer {
private:
    int serverFd, clientFd;
    struct sockaddr_in address;
    std::atomic<bool> running;
    std::thread serverThread;
    
    ILCController* ilc;
    std::atomic<bool>* isSimRunning;
    std::mutex* ilcMutex;
    
public:
    CommandServer(ILCController* ilcPtr, std::atomic<bool>* runFlag, std::mutex* mtx) 
        : serverFd(-1), clientFd(-1), running(false), ilc(ilcPtr), 
          isSimRunning(runFlag), ilcMutex(mtx) {}
    
    ~CommandServer() {
        stop();
    }
    
    void start() {
        serverFd = socket(AF_INET, SOCK_STREAM, 0);
        if (serverFd == 0) {
            std::cerr << "[SERVER] Socket creation failed" << std::endl;
            return;
        }
        
        int opt = 1;
        setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
        
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(SERVER_PORT);
        
        if (bind(serverFd, (struct sockaddr*)&address, sizeof(address)) < 0) {
            std::cerr << "[SERVER] Bind failed" << std::endl;
            return;
        }
        
        if (listen(serverFd, 3) < 0) {
            std::cerr << "[SERVER] Listen failed" << std::endl;
            return;
        }
        
        running = true;
        serverThread = std::thread(&CommandServer::acceptLoop, this);
        
        std::cout << "[SERVER] Command server started on port " << SERVER_PORT << std::endl;
    }
    
    void stop() {
        running = false;
        if (clientFd >= 0) close(clientFd);
        if (serverFd >= 0) close(serverFd);
        if (serverThread.joinable()) serverThread.join();
    }
    
private:
    void acceptLoop() {
        while (running) {
            int addrlen = sizeof(address);
            clientFd = accept(serverFd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
            
            if (clientFd < 0) continue;
            
            std::cout << "[SERVER] Client connected" << std::endl;
            handleClient();
            close(clientFd);
            clientFd = -1;
        }
    }
    
    void handleClient() {
        char buffer[1024] = {0};
        
        while (running) {
            memset(buffer, 0, sizeof(buffer));
            int valread = read(clientFd, buffer, 1024);
            
            if (valread <= 0) break;
            
            std::string command(buffer);
            command.erase(command.find_last_not_of(" \n\r\t") + 1);
            
            std::string response = processCommand(command);
            send(clientFd, response.c_str(), response.length(), 0);
        }
    }
    
    std::string processCommand(const std::string& cmd) {
        std::lock_guard<std::mutex> lock(*ilcMutex);
        
        std::istringstream iss(cmd);
        std::string action;
        iss >> action;
        
        if (action == "start") {
            *isSimRunning = true;
            return "OK: Simulation started\n";
        }
        else if (action == "stop") {
            *isSimRunning = false;
            return "OK: Simulation stopped\n";
        }
        else if (action == "reset") {
            ilc->reset();
            return "OK: ILC reset\n";
        }
        else if (action == "error") {
            double level;
            iss >> level;
            ilc->induceError(level);
            return "OK: Error level set to " + std::to_string(level) + "\n";
        }
        else if (action == "lr") {
            double lr;
            iss >> lr;
            ilc->setLearningRate(lr);
            return "OK: Learning rate set to " + std::to_string(lr) + "\n";
        }
        else if (action == "smooth") {
            double alpha;
            iss >> alpha;
            ilc->setSmoothingFactor(alpha);
            return "OK: Smoothing factor set to " + std::to_string(alpha) + "\n";
        }
        else if (action == "noise") {
            std::string state;
            iss >> state;
            bool enabled = (state == "on" || state == "1");
            ilc->setNoise(enabled);
            return "OK: Noise " + std::string(enabled ? "enabled" : "disabled") + "\n";
        }
        else if (action == "shape") {
            std::string shapeType;
            iss >> shapeType;
            
            std::vector<Point2D> newRef;
            if (shapeType == "circle") {
                double radius = 1.0;
                iss >> radius;
                newRef = ShapeGenerator::generateCircle(NUM_POINTS, radius);
            }
            else if (shapeType == "ellipse") {
                double a = 1.2, b = 0.7;
                iss >> a >> b;
                newRef = ShapeGenerator::generateEllipse(NUM_POINTS, a, b);
            }
            else if (shapeType == "square") {
                double side = 1.6;
                iss >> side;
                newRef = ShapeGenerator::generateSquare(NUM_POINTS, side);
            }
            else if (shapeType == "star") {
                double outer = 1.0, inner = 0.4;
                iss >> outer >> inner;
                newRef = ShapeGenerator::generateStar(NUM_POINTS, outer, inner);
            }
            else {
                return "ERROR: Unknown shape type\n";
            }
            
            ilc->setReference(newRef);
            ilc->reset();
            return "OK: Shape changed to " + shapeType + "\n";
        }
        else if (action == "status") {
            std::ostringstream oss;
            oss << "Iteration: " << ilc->getIteration() << "\n";
            oss << "RMS Error: " << ilc->getRMSError() << "\n";
            oss << "Error Level: " << (ilc->getErrorLevel() * 100) << "%\n";
            oss << "Noise: " << (ilc->getNoiseEnabled() ? "ON" : "OFF") << "\n";
            oss << "Avg Correction: " << ilc->getAvgCorrection() << "\n";
            oss << "Running: " << (*isSimRunning ? "YES" : "NO") << "\n";
            return oss.str();
        }
        else if (action == "help") {
            return "Commands:\n"
                   "  start - Start simulation\n"
                   "  stop - Stop simulation\n"
                   "  reset - Reset ILC\n"
                   "  error <level> - Set error level (0.0-1.0)\n"
                   "  lr <rate> - Set learning rate (0.1-0.8)\n"
                   "  smooth <alpha> - Set smoothing factor (0.1-1.0)\n"
                   "  noise <on|off> - Enable/disable noise\n"
                   "  shape <type> [params] - Change shape (circle, ellipse, square, star)\n"
                   "  status - Get current status\n"
                   "  help - Show this help\n";
        }
        
        return "ERROR: Unknown command\n";
    }
};

// ============================================================================
// MAIN SIMULATOR
// ============================================================================
int main() {
    std::cout << "=== ILC Real-Time Tracker ===" << std::endl;
    std::cout << "Initializing..." << std::endl;
    
    ILCController ilc(NUM_POINTS, 0.4);
    X11Visualizer viz;
    
    std::atomic<bool> isRunning(false);
    std::mutex ilcMutex;
    
    CommandServer server(&ilc, &isRunning, &ilcMutex);
    server.start();
    
    std::cout << "\n[READY] Visualization window opened" << std::endl;
    std::cout << "[READY] Command server listening on port " << SERVER_PORT << std::endl;
    std::cout << "[READY] Use command client to control simulation" << std::endl;
    
    double time = 0;
    int pathIndex = 0;
    Point2D robotPos(1, 0);
    
    auto lastTime = std::chrono::steady_clock::now();
    
    while (true) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
        
        if (elapsed >= 16) { // ~60 FPS
            lastTime = currentTime;
            
            {
                std::lock_guard<std::mutex> lock(ilcMutex);
                
                if (isRunning) {
                    time += 0.8;
                    pathIndex = (int)time % NUM_POINTS;
                    robotPos = ilc.getCurrentPosition(pathIndex);
                    
                    // Check for iteration completion
                    if (pathIndex == 0 && time > NUM_POINTS && 
                        ilc.getCurrentTrajectory().size() >= (size_t)NUM_POINTS) {
                        ilc.completeIteration();
                        time = 0;
                    }
                }
            }
            
            // Render
            viz.clear();
            viz.drawGrid();
            
            {
                std::lock_guard<std::mutex> lock(ilcMutex);
                
                // Reference path
                viz.drawPath(ilc.getReference(), 0, 3, true);
                
                // Completed paths
                const auto& completed = ilc.getCompletedPaths();
                for (const auto& path : completed) {
                    int colorIdx = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawPath(path, colorIdx, 2, true);
                }
                
                // Current trajectory
                if (ilc.getCurrentTrajectory().size() > 2) {
                    int colorIdx = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawPath(ilc.getCurrentTrajectory(), colorIdx, 3, false);
                }
                
                // Robot
                if (isRunning) {
                    int robotColor = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawRobot(robotPos, robotColor, 10);
                }
                
                // Status text
                std::string statusText = "Iteration: " + std::to_string(ilc.getIteration()) + 
                                        " | RMS Error: " + std::to_string(ilc.getRMSError()).substr(0, 8) +
                                        " | Error Level: " + std::to_string((int)(ilc.getErrorLevel() * 100)) + "%";
                viz.drawText(20, 30, statusText);
            }
            
            viz.flush();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    return 0;
}
