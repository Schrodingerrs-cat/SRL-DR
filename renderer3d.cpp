// ============================================================================
// ILC 3D RENDERER (OpenGL + TCP Listener) - WITH BUILD ANIMATION MODES
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
#include <algorithm>

const int PORT = 8766;
const float POINT_SIZE = 4.0f;
const float LINE_WIDTH = 2.0f;

// Simple HSV -> RGB (h in [0,1], s,v in [0,1])
static void hsv2rgb(float h, float s, float v, float &r, float &g, float &b) {
    if (s <= 0.0f) { r = g = b = v; return; }
    h = fmodf(h, 1.0f) * 6.0f;
    int i = (int)floor(h);
    float f = h - i;
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    switch (i) {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        default: r=v; g=p; b=q; break;
    }
}

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
std::mutex dataMutex;

struct Point3D {
    float x, y, z;
};

std::vector<std::vector<Point3D>> pathRings;
int currentReceiveRingSize = 0;
size_t receiveExpectedPoints = 0;
size_t receivedCount = 0;
std::vector<Point3D> receiveBuffer;
bool running = true;
bool paused = false;

// camera state
float rotX = 25.0f, rotY = -45.0f;
float zoom = 2.5f;
float cameraTargetX = 0.0f, cameraTargetY = 0.0f, cameraTargetZ = 0.0f;
float cameraDistance = 2.5f;
int lastX, lastY;
bool dragging = false;

// live cursor animation indices
int drawRingIndex = 0;
int drawVertexIndex = 0;

// build animation mode
enum RenderMode { MODE_LIVE = 0, MODE_BUILD = 1 };
RenderMode renderMode = MODE_LIVE;
int buildRingIndex = -1;       // index of last visible layer in build mode
int buildFrameCounter = 0;     // frames spent on current layer
int framesPerLayer = 8;        // smaller means faster build

// UI button positions and size
struct Button { float x1, y1, x2, y2; std::string label; };
std::vector<Button> buttons;

// Window size tracker
int winW = 900, winH = 700;

// ---------------------------------------------------------------------------
// Recompute View
// ---------------------------------------------------------------------------
void recomputeView() {
    std::lock_guard<std::mutex> lk(dataMutex);
    if (pathRings.empty()) return;

    float minx = 1e9f, miny = 1e9f, minz = 1e9f;
    float maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;

    for (auto &r : pathRings) {
        for (auto &p : r) {
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
            minz = std::min(minz, p.z);
            maxz = std::max(maxz, p.z);
        }
    }

    float cx = 0.5f * (minx + maxx);
    float cy = 0.5f * (miny + maxy);
    float cz = 0.5f * (minz + maxz);

    float dx = maxx - minx;
    float dy = maxy - miny;
    float dz = maxz - minz;
    float span = std::max(std::max(dx, dy), std::max(dz, 1e-3f));

    cameraTargetX = cx;
    cameraTargetY = cy;
    cameraTargetZ = cz;
    cameraDistance = span * 1.5f;
}

// Safe, clamped zoom helper
inline void applyZoomDelta(float delta) {
    zoom += delta;
    if (zoom < 0.2f) zoom = 0.2f;
    if (zoom > 50.0f) zoom = 50.0f;
}

// ---------------------------------------------------------------------------
// TCP SERVER THREAD
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

        std::cout << "[Renderer3D] Simulator connected" << std::endl;

        std::string buffer;
        char buf[2048];
        while (true) {
            ssize_t n = recv(client, buf, sizeof(buf) - 1, 0);
            if (n <= 0) break;
            buf[n] = '\0';
            buffer += buf;

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                line.erase(0, line.find_first_not_of(" \t\r\n"));
                if (!line.empty() && line.find_last_not_of(" \t\r\n") != std::string::npos)
                    line.erase(line.find_last_not_of(" \t\r\n") + 1);
                if (line.empty()) continue;

                if (line.rfind("BEGIN_PATH", 0) == 0) {
                    std::lock_guard<std::mutex> lk(dataMutex);
                    receiveBuffer.clear();
                    receivedCount = 0;

                    std::istringstream iss(line);
                    std::string tag;
                    size_t totalPts;
                    int ringSize;
                    iss >> tag >> totalPts >> ringSize;

                    currentReceiveRingSize = ringSize;
                    receiveExpectedPoints = totalPts;

                    std::cout << "[Renderer3D] BEGIN_PATH: expecting " << totalPts
                              << " points (" << ringSize << " per ring)" << std::endl;
                }
                else if (line.rfind("END_PATH", 0) == 0) {
                    {
                        std::lock_guard<std::mutex> lk(dataMutex);
                        if (!receiveBuffer.empty())
                            pathRings.push_back(receiveBuffer);

                        std::cout << "[Renderer3D] END_PATH — rings: "
                                  << pathRings.size() << std::endl;

                        drawRingIndex = 0;
                        drawVertexIndex = 0;
                        buildRingIndex = -1;
                        buildFrameCounter = 0;
                    }

                    static int redrawCounter = 0;
                    if (++redrawCounter % 3 == 0) {
                        recomputeView();
                        glutPostRedisplay();
                    }
                }
                else if (line == "RESET_VIEW") {
                    std::lock_guard<std::mutex> lk(dataMutex);
                    pathRings.clear();
                    receiveBuffer.clear();
                    currentReceiveRingSize = 0;
                    receiveExpectedPoints = 0;
                    receivedCount = 0;
                    drawRingIndex = 0;
                    drawVertexIndex = 0;
                    buildRingIndex = -1;
                    buildFrameCounter = 0;

                    cameraTargetX = cameraTargetY = cameraTargetZ = 0.0f;
                    rotX = 25.0f;
                    rotY = -45.0f;
                    zoom = cameraDistance = 2.5f;

                    recomputeView();
                    std::cout << "[Renderer3D] Received RESET_VIEW — cleared view"
                              << std::endl;
                    glutPostRedisplay();
                }
                else {
                    std::istringstream iss(line);
                    float x, y, z;
                    if (iss >> x >> y >> z) {
                        std::lock_guard<std::mutex> lk(dataMutex);
                        receiveBuffer.push_back({x, y, z});
                        receivedCount++;

                        if (currentReceiveRingSize > 0 &&
                            (int)receiveBuffer.size() >= currentReceiveRingSize) {
                            pathRings.push_back(receiveBuffer);
                            receiveBuffer.clear();
                        }
                    }
                }
            }
        }

        std::cout << "[Renderer3D] Simulator disconnected" << std::endl;
        close(client);
    }

    close(server_fd);
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------
void drawAxes() {
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(2,0,0); // X
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,2,0); // Y
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,2); // Z
    glEnd();
}

void drawFilledMesh() {
    std::lock_guard<std::mutex> lk(dataMutex);
    if (pathRings.size() < 1) return;

    const float baseR = 0.36f;
    const float baseG = 0.44f;
    const float baseB = 0.54f;

    size_t ringsToDraw = pathRings.size();
    if (renderMode == MODE_BUILD) {
        if (buildRingIndex < 0) {
            return; // nothing built yet
        }
        ringsToDraw = std::min(ringsToDraw,
                               static_cast<size_t>(buildRingIndex + 1));
    }

    for (size_t r = 0; r + 1 < ringsToDraw; ++r) {
        const auto &A = pathRings[r];
        const auto &B = pathRings[r + 1];
        int N = std::min(A.size(), B.size());
        if (N < 2) continue;

        float fade = static_cast<float>(r) /
                     std::max((int)pathRings.size() - 1, 1);
        float R = baseR * (1.0f - 0.3f * fade);
        float G = baseG * (1.0f - 0.3f * fade);
        float Bb = baseB * (1.0f - 0.3f * fade);

        glColor3f(R, G, Bb);
        glLineWidth(LINE_WIDTH);

        glBegin(GL_LINE_STRIP);
        for (int i = 0; i < N; ++i)
            glVertex3f(A[i].x, A[i].y, A[i].z);
        glVertex3f(A[0].x, A[0].y, A[0].z);
        glEnd();

        glBegin(GL_TRIANGLE_STRIP);
        for (int i = 0; i < N; ++i) {
            glColor3f(R, G, Bb);
            glVertex3f(A[i].x, A[i].y, A[i].z);
            glColor3f(R * 0.9f, G * 0.9f, Bb * 0.9f);
            glVertex3f(B[i].x, B[i].y, B[i].z);
        }
        glVertex3f(A[0].x, A[0].y, A[0].z);
        glVertex3f(B[0].x, B[0].y, B[0].z);
        glEnd();
    }

    // cursor point (only meaningful in live mode)
    if (renderMode == MODE_LIVE && drawRingIndex < (int)pathRings.size()) {
        const auto &r = pathRings[drawRingIndex];
        if (!r.empty()) {
            int idx = std::min(drawVertexIndex, (int)r.size() - 1);
            glPointSize(POINT_SIZE * 2.0f);
            glColor3f(1.0f, 1.0f, 0.0f);
            glBegin(GL_POINTS);
            glVertex3f(r[idx].x, r[idx].y, r[idx].z);
            glEnd();
        }
    }
}

// ---------------------------------------------------------------------------
// Overlay Buttons (Zoom UI)
// ---------------------------------------------------------------------------
void drawButtons2D() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, winW, 0, winH);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    for (const auto &b : buttons) {
        // background
        glColor3f(0.15f, 0.15f, 0.2f);
        glBegin(GL_QUADS);
        glVertex2f(b.x1, b.y1);
        glVertex2f(b.x2, b.y1);
        glVertex2f(b.x2, b.y2);
        glVertex2f(b.x1, b.y2);
        glEnd();

        // border
        glColor3f(0.8f, 0.8f, 0.8f);
        glBegin(GL_LINE_LOOP);
        glVertex2f(b.x1, b.y1);
        glVertex2f(b.x2, b.y1);
        glVertex2f(b.x2, b.y2);
        glVertex2f(b.x1, b.y2);
        glEnd();

        // label
        glColor3f(1, 1, 1);
        glRasterPos2f(b.x1 + 12, b.y1 + 8);
        for (char c : b.label)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
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
    drawFilledMesh();
    drawButtons2D();
    glutSwapBuffers();
}

void idle() {
    if (!paused) {
        std::lock_guard<std::mutex> lk(dataMutex);
        if (!pathRings.empty()) {
            if (renderMode == MODE_LIVE) {
                int ringCount = (int)pathRings.size();
                if (drawRingIndex < ringCount) {
                    int ringSize = (int)pathRings[drawRingIndex].size();
                    drawVertexIndex += 3;
                    if (drawVertexIndex >= ringSize) {
                        drawVertexIndex = 0;
                        drawRingIndex++;
                    }
                } else {
                    drawRingIndex = 0;
                    drawVertexIndex = 0;
                }
            } else if (renderMode == MODE_BUILD) {
                size_t totalRings = pathRings.size();
                if (buildRingIndex < (int)totalRings - 1) {
                    buildFrameCounter++;
                    if (buildFrameCounter >= framesPerLayer) {
                        buildRingIndex++;
                        buildFrameCounter = 0;
                    }
                }
            }
        }
    }
    glutPostRedisplay();
}

void reshape(int w, int h) {
    winW = w;
    winH = h;

    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (float)w / (float)h, 0.1, 100.0);

    float bw = 40.0f, bh = 30.0f, pad = 10.0f;
    buttons.clear();
    buttons.push_back({w - 2*bw - 2*pad, pad, w - bw - 2*pad, pad + bh, "-"});
    buttons.push_back({w - bw - pad, pad, w - pad, pad + bh, "+"});
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        int yFlipped = winH - y;

        for (const auto &b : buttons) {
            if (x >= b.x1 && x <= b.x2 && yFlipped >= b.y1 && yFlipped <= b.y2) {
                if (b.label == "+") applyZoomDelta(-0.2f);
                if (b.label == "-") applyZoomDelta(0.2f);
                glutPostRedisplay();
                return;
            }
        }

        dragging = true;
        lastX = x;
        lastY = y;
    }

    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        dragging = false;
    }
}

void motion(int x, int y) {
    if (dragging) {
        rotY += (x - lastX) * 0.5f;
        rotX += (y - lastY) * 0.5f;
        lastX = x;
        lastY = y;
    }
}

void keyboard(unsigned char key, int, int) {
    if (key == 27) {
        running = false;
        exit(0);
    }
    if (key == ' ') {
        paused = !paused;
    }
    if (key == '+' || key == '=') {
        applyZoomDelta(-0.2f);
    }
    if (key == '-' || key == '_') {
        applyZoomDelta(0.2f);
    }
    if (key == 'r' || key == 'R') {
        rotX = 25.0f;
        rotY = -45.0f;
        zoom = 2.5f;
        std::cout << "[Renderer3D] View reset" << std::endl;
    }
    if (key == 'l' || key == 'L') {
        renderMode = MODE_LIVE;
        drawRingIndex = 0;
        drawVertexIndex = 0;
        buildRingIndex = -1;
        buildFrameCounter = 0;
        std::cout << "[Renderer3D] Mode: LIVE (full mesh)" << std::endl;
    }
    if (key == 'g' || key == 'G') {
        renderMode = MODE_BUILD;
        buildRingIndex = -1;
        buildFrameCounter = 0;
        drawRingIndex = 0;
        drawVertexIndex = 0;
        std::cout << "[Renderer3D] Mode: BUILD (layer-by-layer)" << std::endl;
    }
}

// ---------------------------------------------------------------------------
// MAIN
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::cout << "=== ILC 3D Renderer ===" << std::endl;
    std::cout << "Controls: Mouse drag=rotate, plus/minus=zoom, space=pause, R=reset, ESC=exit" << std::endl;
    std::cout << "Modes: L=Live full model, G=Build animation (layer-by-layer)" << std::endl;

    std::thread(serverThreadFunc).detach();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(900, 700);
    glutCreateWindow("ILC 3D Printer Visualizer");

    winW = 900;
    winH = 700;

    float bw = 40.0f, bh = 30.0f, pad = 10.0f;
    buttons = {
        {winW - 2*bw - 2*pad, pad, winW - bw - 2*pad, pad + bh, "-"},
        {winW - bw - pad, pad, winW - pad, pad + bh, "+"}
    };

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glPointSize(POINT_SIZE);

    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);

    renderMode = MODE_LIVE;

    glutMainLoop();
    return 0;
}
