#include <arpa/inet.h>   // inet_addr
#include <sys/types.h>
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
const int WINDOW_SIZE = 800;
const int MARGIN = 60;
const double WORLD_MIN = -1.8;
const double WORLD_MAX = 1.8;
const int NUM_POINTS = 150;
const int SERVER_PORT = 8765;
const int RENDERER_PORT = 8766; // TCP port for external 3D renderer (simulator -> renderer)

// --------------------------- dynamic world bounds ---------------------------
static double WORLD_MIN_X = -1.0;
static double WORLD_MAX_X =  1.0;
static double WORLD_MIN_Y = -1.0;
static double WORLD_MAX_Y =  1.0;



// ============================================================================
// MATH UTILITIES
// ============================================================================
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Point3D {
    double x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
};

// Forward declarations for functions defined later
std::vector<std::vector<Point3D>> generateMorphRings(
    const std::vector<Point2D>& fromRing, const std::vector<Point2D>& toRef,
    int revolutions, double heightPerRevolution);

std::vector<Point3D> flattenRingsToPath(const std::vector<std::vector<Point3D>>& rings);
void writeGCodeForPath(const std::vector<Point3D>& path, std::ofstream& file) {
	 for (auto& p : path)
	 file << "G1 X" << p.x << " Y" << p.y << " Z" << p.z << " F1200\n";
}

inline double distance(const Point2D& a, const Point2D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

// Convert a 2D world coordinate to screen coordinate using dynamic bounds
inline Point2D worldToScreen(double x, double y, int width, int height) {
    double nx = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X);
    double ny = (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y);
    return Point2D(
        MARGIN + nx * (width - 2 * MARGIN),
        MARGIN + (1 - ny) * (height - 2 * MARGIN)
    );
}

// Convenience wrappers for compatibility with old code
inline int worldToScreenX(double x, int width) {
    return static_cast<int>(MARGIN + (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X) * (width - 2 * MARGIN));
}

inline int worldToScreenY(double y, int height) {
    return static_cast<int>(MARGIN + (1 - (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y)) * (height - 2 * MARGIN));
}


// ============================================================================
// SHAPE GENERATOR
// ============================================================================
class ShapeGenerator {
public:
    static std::vector<Point2D> generateCircle(int numPoints, double radius = 1.0) {
        std::vector<Point2D> points;
        for (int i = 0; i < numPoints; i++) {
            double theta = (2.0 * M_PI * i) / numPoints;
            points.push_back(Point2D(radius * std::cos(theta), radius * std::sin(theta)));
        }
        return points;
    }
    
    static std::vector<Point2D> generateEllipse(int numPoints, double a = 1.2, double b = 0.7) {
        std::vector<Point2D> points;
        for (int i = 0; i < numPoints; i++) {
            double theta = (2.0 * M_PI * i) / numPoints;
            points.push_back(Point2D(a * std::cos(theta), b * std::sin(theta)));
        }
        return points;
    }
    
    static std::vector<Point2D> generateSquare(int numPoints, double side = 1.6) {
        std::vector<Point2D> points;
        int pointsPerSide = numPoints / 4;
        double half = side / 2.0;
        
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(-half + side * t, -half));
        }
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(half, -half + side * t));
        }
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(half - side * t, half));
        }
        for (int i = 0; i < pointsPerSide; i++) {
            double t = (double)i / pointsPerSide;
            points.push_back(Point2D(-half, half - side * t));
        }
        
        while (points.size() < (size_t)numPoints) {
            points.push_back(points.back());
        }
        points.resize(numPoints);
        return points;
    }
    
    static std::vector<Point2D> generateStar(int numPoints, double outerRadius = 1.0, double innerRadius = 0.4) {
        std::vector<Point2D> points;
        int peaks = 5;
        int pointsPerPeak = numPoints / (peaks * 2);
        
        for (int i = 0; i < peaks * 2; i++) {
            double angle = (M_PI * i) / peaks;
            double radius = (i % 2 == 0) ? outerRadius : innerRadius;
            
            for (int j = 0; j < pointsPerPeak; j++) {
                double t = (double)j / pointsPerPeak;
                double nextAngle = (M_PI * (i + 1)) / peaks;
                double nextRadius = ((i + 1) % 2 == 0) ? outerRadius : innerRadius;
                
                double r = radius + t * (nextRadius - radius);
                double a = angle + t * (nextAngle - angle);
                points.push_back(Point2D(r * std::cos(a), r * std::sin(a)));
            }
        }
        points.resize(numPoints);
        return points;
    }
    
    
	    // ----------------------------- geometry helpers -----------------------------
    static Point2D computeCentroid(const std::vector<Point2D>& poly) {
        double cx = 0.0, cy = 0.0;
        double area = 0.0;
        size_t n = poly.size();
        if (n == 0) return Point2D(0,0);
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            double a = poly[i].x * poly[j].y - poly[j].x * poly[i].y;
            area += a;
            cx += (poly[i].x + poly[j].x) * a;
            cy += (poly[i].y + poly[j].y) * a;
        }
        if (std::abs(area) < 1e-9) {
            cx = 0; cy = 0;
            for (auto &p: poly) { cx += p.x; cy += p.y; }
            return Point2D(cx / n, cy / n);
        }
        area *= 0.5;
        cx /= (6.0 * area);
        cy /= (6.0 * area);
        return Point2D(cx, cy);
    }

    static std::vector<Point2D> scaleShapeAroundCentroid(const std::vector<Point2D>& shape, double scale) {
        Point2D c = computeCentroid(shape);
        std::vector<Point2D> out; out.reserve(shape.size());
        for (auto &p : shape) {
            out.emplace_back(c.x + (p.x - c.x) * scale, c.y + (p.y - c.y) * scale);
        }
        return out;
    }
    
	// --------------------------- dynamic bounds update ---------------------------
	
    
	// New: smoothly morph one closed shape into another
	static std::vector<Point2D> morphShapes(
	const std::vector<Point2D>& fromShape,
	const std::vector<Point2D>& toShape,
	double t // 0 = fromShape, 1 = toShape
	) {
	std::vector<Point2D> morphed;
	size_t n = std::min(fromShape.size(), toShape.size());
	morphed.reserve(n);
	for (size_t i = 0; i < n; i++) {
	Point2D p1 = fromShape[i];
	Point2D p2 = toShape[i];
	Point2D p((1 - t) * p1.x + t * p2.x,
	(1 - t) * p1.y + t * p2.y);
	morphed.push_back(p);
	}
	return morphed;
	}
	
	

	// Flatten rings -> single path point list (in order), and optionally write G-code
	std::vector<Point3D> flattenRingsToPath(const std::vector<std::vector<Point3D>>& rings) {
	    std::vector<Point3D> path;
	    for (const auto &ring : rings) {
		for (const auto &p : ring) path.push_back(p);
	    }
	    return path;
	}

	void writeGCodeForPath(const std::vector<Point3D>& path, const std::string& filename, double feedrate = 1500.0, double extrusionFactor = 0.02) {
	    std::ofstream ofs(filename);
	    if (!ofs.is_open()) {
		std::cerr << "[GCODE] Failed to open " << filename << std::endl;
		return;
	    }
	    ofs << "; Generated by ILC simulator\n";
	    ofs << "G21 ; mm\nG90 ; absolute\nG92 E0\n";
	    double e = 0.0;
	    ofs << "G1 F" << feedrate << "\n";
	    if (!path.empty()) {
		ofs << "G0 X" << path[0].x << " Y" << path[0].y << " Z" << path[0].z << "\n";
	    }
	    for (size_t i = 1; i < path.size(); ++i) {
		double dx = path[i].x - path[i-1].x;
		double dy = path[i].y - path[i-1].y;
		double dz = path[i].z - path[i-1].z;
		double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
		e += dist * extrusionFactor;
		ofs << "G1 X" << path[i].x << " Y" << path[i].y << " Z" << path[i].z << " E" << e << "\n";
	    }
	    ofs << "M104 S0\nM140 S0\nM84\n";
	    ofs.close();
	    std::cout << "[GCODE] Wrote " << path.size() << " moves to " << filename << std::endl;
	}

};

static void updateWorldBounds(const std::vector<Point2D>& shape) {
	    if (shape.empty()) return;
	    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9;
	    for (const auto& p : shape) {
		minx = std::min(minx, p.x);
		maxx = std::max(maxx, p.x);
		miny = std::min(miny, p.y);
		maxy = std::max(maxy, p.y);
	    }
	    double padx = (maxx - minx) * 0.1 + 0.01; // 10% padding
	    double paddy = (maxy - miny) * 0.1 + 0.01;
	    WORLD_MIN_X = minx - padx;
	    WORLD_MAX_X = maxx + padx;
	    WORLD_MIN_Y = miny - paddy;
	    WORLD_MAX_Y = maxy + paddy;

	    std::cout << "[2D View] Updated world bounds: X[" 
		      << WORLD_MIN_X << ", " << WORLD_MAX_X
		      << "] Y[" << WORLD_MIN_Y << ", " << WORLD_MAX_Y << "]\n";
	}


// ============================================================================
// ILC CONTROLLER
// ============================================================================
class ILCController {
private:
    int numPoints;
    double learningRate;
    double systemErrorLevel;
    int iteration;
    bool enableNoise;
    double smoothingAlpha;  // Smoothing factor for gradual correction
    
    std::vector<Point2D> reference;
    std::vector<Point3D> corrections;
    std::vector<Point2D> currentTrajectory;
    std::vector<Point2D> lastErrors;
    std::vector<std::vector<Point2D>> completedPaths;
    
    double lastRMSError;
    
public:
    ILCController(int nPts, double lr) 
        : numPoints(nPts), learningRate(lr), systemErrorLevel(0.0), 
          iteration(0), enableNoise(false), smoothingAlpha(0.3), lastRMSError(0.0) {
        corrections.assign(numPoints, Point3D(0, 0, 0));
        reference = ShapeGenerator::generateCircle(numPoints);
    }
    
    void setReference(const std::vector<Point2D>& ref) {
        reference = ref;
        updateWorldBounds(ref);
        reference.resize(numPoints);
    }
    
        // New: gradually transform reference shape into a new one over multiple iterations
    void morphReferenceTo(const std::vector<Point2D>& newShape, double progress) {
        // Clamp progress 0–1
        progress = std::max(0.0, std::min(1.0, progress));
        auto morphed = ShapeGenerator::morphShapes(reference, newShape, progress);
        reference = morphed;
    }

    // New: build dome layers by reducing radius each iteration
    void buildDomeLayered(double initialRadius, double finalRadius, int totalLayers) {
        double step = (initialRadius - finalRadius) / std::max(1, totalLayers - 1);
        completedPaths.clear();

        for (int i = 0; i < totalLayers; i++) {
            double currentRadius = initialRadius - i * step;
            auto layer = ShapeGenerator::generateCircle(numPoints, currentRadius);
            completedPaths.push_back(layer);
        }
        // After dome layers are generated, set the last layer as new reference
	if (!completedPaths.empty()) {
    	reference = completedPaths.back();
	}
        std::cout << "[ILC] Dome layers generated (" << totalLayers << " layers)\n";
    }

    
    Point2D plantModel(const Point2D& command, int pathIndex) {
        Point2D output = command;
        
        if (systemErrorLevel > 0) {
            double theta = (2.0 * M_PI * pathIndex) / numPoints;
            
            // Multi-harmonic systematic error
            double radialError = systemErrorLevel * (
                0.25 * std::sin(3 * theta) +
                0.15 * std::sin(5 * theta) +
                0.10 * std::cos(2 * theta)
            );
            
            double phaseError = systemErrorLevel * (0.18 + 0.02 * std::sin(0.5 * theta + iteration));
            double contractionError = systemErrorLevel * (0.12 + 0.03 * std::cos(1.5 * theta));
            
            double radius = std::sqrt(output.x * output.x + output.y * output.y);
            double angle = std::atan2(output.y, output.x);
            
            double newRadius = radius * (1 - contractionError) + radialError;
            double newAngle = angle + phaseError;
            
            output.x = newRadius * std::cos(newAngle);
            output.y = newRadius * std::sin(newAngle);
            
            // Optional noise
            if (enableNoise) {
                double noiseAmp = systemErrorLevel * 0.04;
                output.x += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
                output.y += (rand() / (double)RAND_MAX - 0.5) * noiseAmp;
            }
        }
        
        return output;
    }
    
    Point2D getCurrentPosition(int pathIndex) {
        if (pathIndex >= numPoints) pathIndex = numPoints - 1;
        
        Point2D refPoint = reference[pathIndex];
        Point2D correction(corrections[pathIndex].x, corrections[pathIndex].y);
        
        // Apply full correction (no smoothing here - that's for the update law)
        Point2D command;
        command.x = refPoint.x + correction.x;
        command.y = refPoint.y + correction.y;
        
        Point2D actual = plantModel(command, pathIndex);
        
        if ((int)currentTrajectory.size() <= pathIndex) {
            currentTrajectory.push_back(actual);
        } else {
            currentTrajectory[pathIndex] = actual;
        }
        
        return actual;
    }
    
    double completeIteration() {
        if (currentTrajectory.size() < (size_t)numPoints) return lastRMSError;
        
        std::cout << "[ILC] Completing iteration " << iteration << std::endl;
        
        std::vector<Point2D> trackingErrors;
        double totalError = 0;
        
        // Standard ILC: Calculate error from REFERENCE, not previous iteration
        for (int i = 0; i < numPoints; i++) {
            Point2D refPt = reference[i];
            Point2D actualPt = currentTrajectory[i];
            
            // Direct error calculation: e_k(i) = reference(i) - actual_k(i)
            Point2D error;
            error.x = refPt.x - actualPt.x;
            error.y = refPt.y - actualPt.y;
            trackingErrors.push_back(error);
            
            totalError += error.x * error.x + error.y * error.y;
        }
        
        double rmsError = std::sqrt(totalError / numPoints);
        lastRMSError = rmsError;
        
        // Standard ILC update with smoothing for gradual correction
        // u_{k+1}(i) = u_k(i) + α * L * e_k(i)
        // where e_k(i) = reference(i) - actual_k(i)
        // 
        // Smoothing approach: blend new correction with old
        // This prevents abrupt changes in the correction signal
        for (int i = 0; i < numPoints && i < (int)trackingErrors.size(); i++) {
            // Calculate proposed correction
            double proposedDeltaX = learningRate * trackingErrors[i].x;
            double proposedDeltaY = learningRate * trackingErrors[i].y;
            
            // Smooth blending: move gradually toward the new correction
            // α controls how much of the new correction to accept
            // Low α = very smooth, gradual changes
            // High α = accept more of new correction, faster convergence
            corrections[i].x += smoothingAlpha * proposedDeltaX;
            corrections[i].y += smoothingAlpha * proposedDeltaY;
            corrections[i].z += smoothingAlpha * 0.0; // placeholder for Z correction (optional)
            
            // Limit corrections
            double correctionMag = std::sqrt(corrections[i].x * corrections[i].x + 
                                            corrections[i].y * corrections[i].y);
            if (correctionMag > 1.0) {
                corrections[i].x = (corrections[i].x / correctionMag) * 1.0;
                corrections[i].y = (corrections[i].y / correctionMag) * 1.0;
            }
        }
        
        iteration++;
        lastErrors = trackingErrors;
        
        std::cout << "[ILC] Iteration " << iteration-1 << " RMS Error: " << rmsError << std::endl;
        
        // Store completed path
        completedPaths.push_back(currentTrajectory);
        if (completedPaths.size() > 5) {
            completedPaths.erase(completedPaths.begin());
        }
        
        currentTrajectory.clear();
        
        return rmsError;
    }
    
    void induceError(double level) {
        systemErrorLevel = level;
        std::cout << "[ILC] System error set to " << (level * 100) << "%" << std::endl;
    }
    
    void setLearningRate(double lr) {
        learningRate = lr;
        std::cout << "[ILC] Learning rate set to " << lr << std::endl;
    }
    
    void setSmoothingFactor(double alpha) {
        smoothingAlpha = std::max(0.1, std::min(1.0, alpha));
        std::cout << "[ILC] Smoothing factor set to " << smoothingAlpha << std::endl;
        std::cout << "      (Controls rate of correction update - higher = faster learning)" << std::endl;
    }
    
    void setNoise(bool enabled) {
        enableNoise = enabled;
        std::cout << "[ILC] Noise " << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    void reset() {
        corrections.assign(numPoints, Point3D(0, 0, 0));
        currentTrajectory.clear();
        lastErrors.clear();
        completedPaths.clear();
        iteration = 0;
        systemErrorLevel = 0;
        lastRMSError = 0;
        std::cout << "[ILC] Reset complete" << std::endl;
    }
    
    // Getters
    const std::vector<Point2D>& getReference() const { return reference; }
    const std::vector<Point2D>& getCurrentTrajectory() const { return currentTrajectory; }
    const std::vector<std::vector<Point2D>>& getCompletedPaths() const { return completedPaths; }
    const std::vector<Point2D>& getLastErrors() const { return lastErrors; }
    int getIteration() const { return iteration; }
    double getRMSError() const { return lastRMSError; }
    double getErrorLevel() const { return systemErrorLevel; }
    bool getNoiseEnabled() const { return enableNoise; }
    const std::vector<Point2D>& getBaseCrossSection() const {
    return reference;
}

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
            int xPos = worldToScreenX(val, WINDOW_SIZE);
    	    int yPos = worldToScreenY(val, WINDOW_SIZE);
            XDrawLine(display, window, gc, xPos, MARGIN, xPos, WINDOW_SIZE - MARGIN);
    	    XDrawLine(display, window, gc, MARGIN, yPos, WINDOW_SIZE - MARGIN, yPos);
        }
    }
    
    void drawPath(const std::vector<Point2D>& path, int colorIdx, int lineWidth, bool closed = true) {
        if (path.size() < 2) return;
        
        XSetForeground(display, gc, colors[colorIdx].pixel);
        XSetLineAttributes(display, gc, lineWidth, LineSolid, CapRound, JoinRound);
        
        for (size_t i = 0; i < path.size() - 1; i++) {
            Point2D s1 = worldToScreen(path[i].x, path[i].y, WINDOW_SIZE, WINDOW_SIZE);
	    Point2D s2 = worldToScreen(path[i+1].x, path[i+1].y, WINDOW_SIZE, WINDOW_SIZE);
	    XDrawLine(display, window, gc, s1.x, s1.y, s2.x, s2.y);
        }
        
        if (closed && path.size() > 2) {
		int x1 = worldToScreenX(path.back().x, WINDOW_SIZE);
		int y1 = worldToScreenY(path.back().y, WINDOW_SIZE);
		int x2 = worldToScreenX(path[0].x, WINDOW_SIZE);
		int y2 = worldToScreenY(path[0].y, WINDOW_SIZE);
		XDrawLine(display, window, gc, x1, y1, x2, y2);
        }
    }
    
    void drawRobot(const Point2D& pos, int colorIdx, int radius = 10) {
	Point2D s = worldToScreen(pos.x, pos.y, WINDOW_SIZE, WINDOW_SIZE);
	int x = static_cast<int>(s.x);
	int y = static_cast<int>(s.y);

        
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
// RENDERER CLIENT (simulator -> external renderer via TCP)
// ============================================================================
//
// Simple, non-blocking-ish client used to send path data (BEGIN_PATH / END_PATH)
// to a separate renderer executable. If renderer not connected, send is skipped.
//
// NOTE: This keeps the simulator independent of OpenGL/GLUT and leaves 2D X11
// visualization untouched.
//

class RendererClient {
private:
    int sockfd;
    std::mutex sockMutex;
    bool connected;

public:
    RendererClient() : sockfd(-1), connected(false) {}

    ~RendererClient() {
        disconnect();
    }

    // Attempt to connect (non-fatal if renderer absent)
    bool connectToRenderer(const std::string& host = "127.0.0.1", int port = RENDERER_PORT) {
        std::lock_guard<std::mutex> lk(sockMutex);
        if (connected) return true;

        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            std::cerr << "[RendererClient] socket() failed\n";
            return false;
        }

        struct sockaddr_in serv_addr;
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        serv_addr.sin_addr.s_addr = inet_addr(host.c_str());

        // set a short connect timeout (non-blocking approach)
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
        setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const char*)&tv, sizeof tv);

        if (::connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            // failed to connect: close and mark disconnected
            close(sockfd);
            sockfd = -1;
            // Not printing too loudly - renderer may not be running.
            return false;
        }

        connected = true;
        std::cout << "[RendererClient] Connected to renderer on port " << port << std::endl;
        return true;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lk(sockMutex);
        if (sockfd >= 0) {
            close(sockfd);
            sockfd = -1;
        }
        connected = false;
    }

    bool isConnected() {
        std::lock_guard<std::mutex> lk(sockMutex);
        return connected;
    }


    // -------------------------------------------------------------------------
    // Send 3D rings: each ring is a full revolution of the cross-section (x,y,z)
    // Header: "BEGIN_PATH <totalPts> <ringSize>\n"
    // Then stream lines "x y z\n" for all points, grouped by rings.
    // -------------------------------------------------------------------------
bool sendPath3D(const std::vector<std::vector<Point3D>>& rings) {
    std::lock_guard<std::mutex> lk(sockMutex);
    if (!connected || sockfd < 0) return false;

    size_t totalPts = 0;
    int ringSize = rings.empty() ? 0 : (int)rings[0].size();
    for (const auto &r : rings) totalPts += r.size();

    std::ostringstream header;
    header << "BEGIN_PATH " << totalPts << " " << ringSize << "\n";

    // footer declared up front so no initialization is jumped over
    std::string footer = "END_PATH\n";

    // send header
    if (send(sockfd, header.str().c_str(), header.str().size(), 0) < 0) {
        std::cerr << "[RendererClient] send() failed (header)\n";
        close(sockfd);
        sockfd = -1;
        connected = false;
        return false;
    }

    // send body
    for (const auto &ring : rings) {
        for (const auto &p : ring) {
            std::ostringstream oss;
            oss << p.x << " " << p.y << " " << p.z << "\n";
            std::string msg = oss.str();
            if (send(sockfd, msg.c_str(), msg.size(), 0) < 0) {
                std::cerr << "[RendererClient] send() failed (body)\n";
                close(sockfd);
                sockfd = -1;
                connected = false;
                return false;
            }
            usleep(500);
        }
    }

    // send footer
    if (send(sockfd, footer.c_str(), footer.size(), 0) < 0) {
        std::cerr << "[RendererClient] send() failed (footer)\n";
        close(sockfd);
        sockfd = -1;
        connected = false;
        return false;
    }

    return true;
}



    bool sendPath(const std::vector<std::vector<Point2D>>& layers2d) {
        std::vector<std::vector<Point3D>> rings;
        for (const auto &l : layers2d) {
            std::vector<Point3D> ring;
            ring.reserve(l.size());
            for (const auto &p: l)
                ring.emplace_back(p.x, p.y, 0.0);
            rings.push_back(std::move(ring));
        }
        return sendPath3D(rings);
    }
};

static RendererClient rendererClient;








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
	// Before switching reference, morph smoothly from previous shape
	if (!ilc->getCompletedPaths().empty()) {
	    const auto &lastPath = ilc->getCompletedPaths().back();
	    if (!lastPath.empty()) {
		std::cout << "[MORPH] Transitioning from current to new shape...\n";
		auto last2D = lastPath; // reuse last path’s 2D
		auto morphRings = generateMorphRings(last2D, newRef, 12, 0.01);
		rendererClient.sendPath3D(morphRings);
		writeGCodeForPath(flattenRingsToPath(morphRings), "morph_transition.gcode");
	    }
	}

            ilc->setReference(newRef);
            ilc->reset();
            return "OK: Shape changed to " + shapeType + "\n";
        }
       
        else if (action == "morph") {
            std::string shapeType;
            double progress = 0.5;
            iss >> shapeType >> progress;

            std::vector<Point2D> target;
            if (shapeType == "circle") target = ShapeGenerator::generateCircle(NUM_POINTS);
            else if (shapeType == "ellipse") target = ShapeGenerator::generateEllipse(NUM_POINTS);
            else if (shapeType == "square") target = ShapeGenerator::generateSquare(NUM_POINTS);
            else if (shapeType == "star") target = ShapeGenerator::generateStar(NUM_POINTS);
            else return "ERROR: Unknown morph shape\n";

            ilc->morphReferenceTo(target, progress);
            return "OK: Morphing reference toward " + shapeType + " (progress=" + std::to_string(progress) + ")\n";
        }

        else if (action == "dome") {
            double r1 = 1.0, r2 = 0.3;
            int layers = 10;
            iss >> r1 >> r2 >> layers;
            ilc->buildDomeLayered(r1, r2, layers);
            return "OK: Dome layers generated from " + std::to_string(r1) + " → " +
                   std::to_string(r2) + " (" + std::to_string(layers) + " layers)\n";
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
// SPIRAL PATH GENERATOR
// ============================================================================
// Generates a continuous spiral path by tracing the input 2D cross-section and
// gradually increasing z as we move along the contour.
// Optionally shrinks the cross-section for dome formation.
// Returns rings: vector of rings, each ring is vector<Point3D> (one revolution)
std::vector<std::vector<Point3D>> generateSpiralRingsFromCrossSection(
    const std::vector<Point2D>& crossSection,
    double heightPerRevolution,
    int numRevolutions,
    bool makeDome = false)
{
    std::vector<std::vector<Point3D>> rings;
    if (crossSection.empty()) return rings;

    int N = (int)crossSection.size();
    double dz_per_point = heightPerRevolution / (double)N;

    for (int rev = 0; rev < numRevolutions; ++rev) {
        double t = (double)rev / std::max(1, numRevolutions-1); // 0..1
        double scale = 1.0;
        if (makeDome) {
            // smooth shrink: use ease-in curve for nicer dome
            double s = 1.0 - t;
            scale = s * s * (3 - 2 * s); // smoothstep(s)
        }
        std::vector<Point2D> scaled2d = ShapeGenerator::scaleShapeAroundCentroid(crossSection, scale);

        std::vector<Point3D> ring; ring.reserve(N);
        for (int i = 0; i < N; ++i) {
            double x = scaled2d[i].x;
            double y = scaled2d[i].y;
            double z = (rev * N + i) * dz_per_point;
            ring.emplace_back(x, y, z);
        }
        rings.push_back(std::move(ring));
    }
    return rings;
}

// ============================================================================
// MORPH RING GENERATOR (for smooth transition between shapes)
// ============================================================================
std::vector<std::vector<Point3D>> generateMorphRings(
    const std::vector<Point2D>& fromRing, const std::vector<Point2D>& toRef,
    int revolutions, double heightPerRevolution)
{
    std::vector<std::vector<Point3D>> rings;
    int N = (int)std::min(fromRing.size(), toRef.size());
    for (int rev = 0; rev < revolutions; ++rev) {
        double progress = (double)rev / std::max(1, revolutions-1);
        double s = progress * progress * (3 - 2 * progress); // smoothstep
        std::vector<Point2D> interpolated; interpolated.reserve(N);
        for (int i = 0; i < N; ++i) {
            Point2D a = fromRing[i % fromRing.size()];
            Point2D b = toRef[i % toRef.size()];
            interpolated.emplace_back((1-s)*a.x + s*b.x, (1-s)*a.y + s*b.y);
        }
        std::vector<Point3D> ring;
        for (int i=0; i<N; ++i) {
            double z = (rev * N + i) * (heightPerRevolution / N);
            ring.emplace_back(interpolated[i].x, interpolated[i].y, z);
        }
        rings.push_back(std::move(ring));
    }
    return rings;
}

std::vector<Point3D> flattenRingsToPath(const std::vector<std::vector<Point3D>>& rings) {
    std::vector<Point3D> path;
    for (const auto &ring : rings) {
        for (const auto &p : ring) path.push_back(p);
    }
    return path;
}

void writeGCodeForPath(const std::vector<Point3D>& path, const std::string& filename,
                       double feedrate, double extrusionFactor) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "[GCODE] Failed to open " << filename << std::endl;
        return;
    }
    ofs << "; Generated by ILC simulator\n";
    ofs << "G21 ; mm\nG90 ; absolute\nG92 E0\n";
    double e = 0.0;
    ofs << "G1 F" << feedrate << "\n";
    if (!path.empty()) {
        ofs << "G0 X" << path[0].x << " Y" << path[0].y << " Z" << path[0].z << "\n";
    }
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double dz = path[i].z - path[i-1].z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        e += dist * extrusionFactor;
        ofs << "G1 X" << path[i].x << " Y" << path[i].y << " Z" << path[i].z << " E" << e << "\n";
    }
    ofs << "M104 S0\nM140 S0\nM84\n";
    ofs.close();
    std::cout << "[GCODE] Wrote " << path.size() << " moves to " << filename << std::endl;
}

// ============================================================================
// MAIN SIMULATOR
// ============================================================================
int main() {
    std::cout << "=== ILC Real-Time Tracker ===" << std::endl;
    std::cout << "Initializing..." << std::endl;
    
    ILCController ilc(NUM_POINTS, 0.4);
    X11Visualizer viz;
    
    // ✅ Initialize connection to external 3D renderer (separate process)
    // We'll attempt to connect once at startup. If the renderer is not present,
    // simulator keeps running and will try to send if a user later starts renderer.
    rendererClient.connectToRenderer("127.0.0.1", RENDERER_PORT);

    
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

                    // ✅ Update the 3D view here so it syncs with new layers
                    std::cout << "\n[DEBUG] Checking 3D" << std::endl;
                    // Try to send 3D data to external renderer (non-fatal if renderer absent)
		    // Generate a continuous spiral path for 3D rendering
		// ✅ Smooth spiral building for 3D visualization (will be interpreted by renderer)
		bool domePhaseActive = false; // placeholder for now; true when building dome
		std::vector<Point2D> baseCross = ilc.getBaseCrossSection();
		// Build rings (revolutions) from the base cross-section and stream to renderer
		int numRevolutions = 60;      // number of revolutions / rings to produce (tune as needed)
		double heightPerRevolution = 0.01; // vertical rise per revolution (tune to desired layer height)

		// Generate rings (each ring is a full contour at a slightly different scale / z)
		auto rings = generateSpiralRingsFromCrossSection(baseCross, heightPerRevolution, numRevolutions, domePhaseActive);

		// Send structured 3D rings to renderer (streamed, with small delays for visible animation)
		if (!rings.empty()) {
		    rendererClient.sendPath3D(rings);
		}

		// Also write a G-code file for this path so it can be replayed offline
		auto fullPath = flattenRingsToPath(rings);
		static int gcount = 0;
		static std::ofstream gcodeFile("ilc_path_combined.gcode", std::ios::app);
		writeGCodeForPath(fullPath, gcodeFile);
		        }
		    }
            
            // 2D rendering
            viz.clear();
            viz.drawGrid();
            
            {
                std::lock_guard<std::mutex> lock(ilcMutex);
                
                viz.drawPath(ilc.getReference(), 0, 3, true);
                
                const auto& completed = ilc.getCompletedPaths();
                for (const auto& path : completed) {
                    int colorIdx = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawPath(path, colorIdx, 2, true);
                }
                
                if (ilc.getCurrentTrajectory().size() > 2) {
                    int colorIdx = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawPath(ilc.getCurrentTrajectory(), colorIdx, 3, false);
                }
                
                if (isRunning) {
                    int robotColor = (ilc.getErrorLevel() > 0) ? 2 : 1;
                    viz.drawRobot(robotPos, robotColor, 10);
                }
                
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

