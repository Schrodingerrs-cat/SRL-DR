// ============================================================================
// ILC 3D RENDERER (OpenGL + TCP Listener) - FIXED VERSION
// ============================================================================
// Listens on port 8766 for "BEGIN_PATH ... END_PATH" data from ilc_simulator.
// Renders the received (x,y,z) points as a continuous toolpath (spiral build).
//
// Controls:
//   • Mouse drag  -> rotate
//   • Scroll/wheel -> zoom (or keys '+' / '-')
//   • Space       -> pause/resume animation
// ============================================================================

#include <GL/glut.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

const int PORT = 8766;
const float POINT_SIZE = 4.0f;
const float LINE_WIDTH  = 2.0f;

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
std::mutex dataMutex;
std::vector<std::vector<float>> pathPoints; // {x,y,z}
bool running = true;
bool paused  = false;

// view control
float rotX = 25.0f, rotY = -45.0f;
float zoom = 2.5f;
int lastX, lastY;
bool dragging = false;

// animation
int drawIndex = 0;

// ---------------------------------------------------------------------------
// TCP SERVER THREAD - FIXED VERSION
// ---------------------------------------------------------------------------
void serverThreadFunc() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("[Renderer3D] socket");
        return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[Renderer3D] bind");
        close(server_fd);
        return;
    }

    listen(server_fd, 1);
    std::cout << "[Renderer3D] Listening on port " << PORT << " ..." << std::endl;

    while (running) {
        int client = accept(server_fd, nullptr, nullptr);
        if (client < 0) continue;
        std::cout << "[Renderer3D] Simulator connected\n";

        std::string buffer;
        char buf[2048];
        
        while (true) {
            ssize_t n = recv(client, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            buffer += buf;

            // Process complete lines
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                
                // Trim whitespace
                line.erase(0, line.find_first_not_of(" \t\r\n"));
                line.erase(line.find_last_not_of(" \t\r\n") + 1);
                
                if (line.empty()) continue;
                
                std::cout << "[Renderer3D] Line: " << line << std::endl;

                if (line.rfind("BEGIN_PATH", 0) == 0) {
                    std::lock_guard<std::mutex> lk(dataMutex);
                    pathPoints.clear();
                    drawIndex = 0;
                    std::cout << "[Renderer3D] BEGIN_PATH received, cleared buffer\n";
                    
                } else if (line.rfind("END_PATH", 0) == 0) {
                    {
                        std::lock_guard<std::mutex> lk(dataMutex);
                        std::cout << "[Renderer3D] END_PATH - Total points received: "
                                  << pathPoints.size() << "\n";
                    }
                    glutPostRedisplay();
                    
                } else {
                    // Try to parse as "x y z"
                    std::istringstream iss(line);
                    float x, y, z;
                    if (iss >> x >> y >> z) {
                        std::lock_guard<std::mutex> lk(dataMutex);
                        pathPoints.push_back({x, y, z});
                        
                        // Debug: print every 50th point
                        if (pathPoints.size() % 50 == 0) {
                            std::cout << "[Renderer3D] Received " << pathPoints.size() 
                                      << " points (last: " << x << ", " << y << ", " << z << ")\n";
                        }
                    } else {
                        std::cout << "[Renderer3D] Failed to parse line: " << line << std::endl;
                    }
                }
            }
        }

        std::cout << "[Renderer3D] Simulator disconnected\n";
        close(client);
    }

    close(server_fd);
}

// ---------------------------------------------------------------------------
// OpenGL helpers
// ---------------------------------------------------------------------------
void drawAxes() {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X-red
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(2,0,0);
    // Y-green
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,2,0);
    // Z-blue
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,2);
    glEnd();
}

void drawPath() {
    std::lock_guard<std::mutex> lk(dataMutex);
    if (pathPoints.empty()) return;

    glLineWidth(LINE_WIDTH);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < drawIndex && i < (int)pathPoints.size(); ++i) {
        float t = (float)i / (float)pathPoints.size();
        glColor3f(1.0f - t, 0.5f * t, t);  // color gradient
        glVertex3f(pathPoints[i][0], pathPoints[i][1], pathPoints[i][2]);
    }
    glEnd();

    // draw nozzle (current position)
    if (drawIndex > 0 && drawIndex <= (int)pathPoints.size()) {
        glPointSize(POINT_SIZE * 2.0f);
        glColor3f(1, 1, 0);
        glBegin(GL_POINTS);
        int idx = std::min(drawIndex - 1, (int)pathPoints.size() - 1);
        auto &p = pathPoints[idx];
        glVertex3f(p[0], p[1], p[2]);
        glEnd();
    }
}

// ---------------------------------------------------------------------------
// GLUT callbacks
// ---------------------------------------------------------------------------
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -zoom);
    glRotatef(rotX, 1, 0, 0);
    glRotatef(rotY, 0, 1, 0);

    drawAxes();
    drawPath();

    glutSwapBuffers();
}

void idle() {
    if (!paused) {
        std::lock_guard<std::mutex> lk(dataMutex);
        if (!pathPoints.empty() && drawIndex < (int)pathPoints.size()) {
            drawIndex += 2; // speed of animation (adjust as needed)
            if (drawIndex > (int)pathPoints.size()) {
                drawIndex = (int)pathPoints.size();
            }
        }
    }
    glutPostRedisplay();
}

void reshape(int w, int h) {
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)w/h, 0.1, 100.0);
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        dragging = (state == GLUT_DOWN);
        lastX = x; lastY = y;
    }
}

void motion(int x, int y) {
    if (dragging) {
        rotY += (x - lastX) * 0.5f;
        rotX += (y - lastY) * 0.5f;
        lastX = x; lastY = y;
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 27) { running = false; exit(0); }
    if (key == ' ') paused = !paused;
    if (key == '+' || key == '=') zoom -= 0.2f;
    if (key == '-' || key == '_') zoom += 0.2f;
    if (key == 'r' || key == 'R') {
        // Reset view
        rotX = 25.0f;
        rotY = -45.0f;
        zoom = 2.5f;
        std::cout << "[Renderer3D] View reset\n";
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::cout << "=== ILC 3D Renderer ===\n";
    std::cout << "Controls:\n";
    std::cout << "  Mouse drag: Rotate view\n";
    std::cout << "  +/- : Zoom in/out\n";
    std::cout << "  Space: Pause/resume animation\n";
    std::cout << "  R: Reset view\n";
    std::cout << "  ESC: Exit\n\n";
    
    std::thread(serverThreadFunc).detach();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 700);
    glutCreateWindow("ILC 3D Printer Visualizer");

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);  // Dark background
    glPointSize(POINT_SIZE);

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
