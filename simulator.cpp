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

// ============================================================================
// MATH UTILITIES
// ============================================================================
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

inline double distance(const Point2D& a, const Point2D& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx*dx + dy*dy);
}

inline int worldToScreen(double val) {
    return static_cast<int>(MARGIN + (val - WORLD_MIN) / (WORLD_MAX - WORLD_MIN) * (WINDOW_SIZE - 2*MARGIN));
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
};

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
    std::vector<Point2D> corrections;
    std::vector<Point2D> currentTrajectory;
    std::vector<Point2D> lastErrors;
    std::vector<std::vector<Point2D>> completedPaths;
    
    double lastRMSError;
    
public:
    ILCController(int nPts, double lr) 
        : numPoints(nPts), learningRate(lr), systemErrorLevel(0.0), 
          iteration(0), enableNoise(false), smoothingAlpha(0.3), lastRMSError(0.0) {
        corrections.resize(numPoints, Point2D(0, 0));
        reference = ShapeGenerator::generateCircle(numPoints);
    }
    
    void setReference(const std::vector<Point2D>& ref) {
        reference = ref;
        reference.resize(numPoints);
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
        Point2D correction = corrections[pathIndex];
        
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
        corrections.assign(numPoints, Point2D(0, 0));
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
