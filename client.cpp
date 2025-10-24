#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>

const int SERVER_PORT = 8765;
const char* SERVER_IP = "127.0.0.1";

class ILCClient {
private:
    int sock;
    struct sockaddr_in serv_addr;
    bool connected;
    
public:
    ILCClient() : sock(-1), connected(false) {}
    
    ~ILCClient() {
        disconnect();
    }
    
    bool connect() {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation error" << std::endl;
            return false;
        }
        
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(SERVER_PORT);
        
        if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address" << std::endl;
            return false;
        }
        
        if (::connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "Connection failed. Is the simulator running?" << std::endl;
            return false;
        }
        
        connected = true;
        std::cout << "Connected to ILC simulator on port " << SERVER_PORT << std::endl;
        return true;
    }
    
    void disconnect() {
        if (sock >= 0) {
            close(sock);
            sock = -1;
        }
        connected = false;
    }
    
    std::string sendCommand(const std::string& command) {
        if (!connected) {
            return "ERROR: Not connected to simulator\n";
        }
        
        send(sock, command.c_str(), command.length(), 0);
        
        char buffer[4096] = {0};
        int valread = read(sock, buffer, 4096);
        
        if (valread <= 0) {
            return "ERROR: Connection lost\n";
        }
        
        return std::string(buffer);
    }
    
    bool isConnected() const { return connected; }
};

void printHelp() {
    std::cout << "\n=== ILC Command Client ===\n";
    std::cout << "Available commands:\n";
    std::cout << "  start                    - Start the simulation\n";
    std::cout << "  stop                     - Stop the simulation\n";
    std::cout << "  reset                    - Reset ILC (clear all learning)\n";
    std::cout << "  error <level>            - Set system error level (0.0 to 1.0)\n";
    std::cout << "                             Example: error 0.3\n";
    std::cout << "  lr <rate>                - Set learning rate (0.1 to 0.8)\n";
    std::cout << "                             Example: lr 0.5\n";
    std::cout << "  smooth <alpha>           - Set smoothing factor (0.1 to 1.0)\n";
    std::cout << "                             Example: smooth 0.3\n";
    std::cout << "  noise on|off             - Enable/disable random noise\n";
    std::cout << "                             Example: noise on\n";
    std::cout << "  shape <type> [params]    - Change reference shape\n";
    std::cout << "      circle [radius]      - Circle (default radius=1.0)\n";
    std::cout << "                             Example: shape circle 1.2\n";
    std::cout << "      ellipse [a] [b]      - Ellipse (default a=1.2, b=0.7)\n";
    std::cout << "                             Example: shape ellipse 1.5 0.8\n";
    std::cout << "      square [side]        - Square (default side=1.6)\n";
    std::cout << "                             Example: shape square 1.8\n";
    std::cout << "      star [outer] [inner] - Star (default outer=1.0, inner=0.4)\n";
    std::cout << "                             Example: shape star 1.2 0.5\n";
    std::cout << "  status                   - Show current simulation status\n";
    std::cout << "  help                     - Show this help\n";
    std::cout << "  quit                     - Exit client\n";
    std::cout << "\nPreset Scenarios:\n";
    std::cout << "  preset drift             - Mild drift error (error 0.25, noise off)\n";
    std::cout << "  preset lag               - Phase lag error (error 0.45, noise off)\n";
    std::cout << "  preset deform            - Severe deformation (error 0.65, noise off)\n";
    std::cout << "  preset noise             - Stochastic disturbance (error 0.4, noise on)\n";
    std::cout << "\n";
    std::cout << "  morph <type> <progress>   - Gradually morph current shape toward another\n";
    std::cout << "                             Example: morph circle 0.5\n";
    std::cout << "  dome <r1> <r2> <layers>   - Build layered dome of shrinking circles\n";
    std::cout << "                             Example: dome 1.5 0.3 10\n";

}

void handlePreset(ILCClient& client, const std::string& preset) {
    if (preset == "drift") {
        std::cout << client.sendCommand("error 0.25");
        std::cout << client.sendCommand("noise off");
        std::cout << "Preset 'drift' applied: 25% error, no noise\n";
    }
    else if (preset == "lag") {
        std::cout << client.sendCommand("error 0.45");
        std::cout << client.sendCommand("noise off");
        std::cout << "Preset 'lag' applied: 45% error, no noise\n";
    }
    else if (preset == "deform") {
        std::cout << client.sendCommand("error 0.65");
        std::cout << client.sendCommand("noise off");
        std::cout << "Preset 'deform' applied: 65% error, no noise\n";
    }
    else if (preset == "noise") {
        std::cout << client.sendCommand("error 0.4");
        std::cout << client.sendCommand("noise on");
        std::cout << "Preset 'noise' applied: 40% error, noise enabled\n";
    }
    else {
        std::cout << "Unknown preset. Available: drift, lag, deform, noise\n";
    }
}

int main() {
    std::cout << "=== ILC Simulator Command Client ===\n";
    std::cout << "Connecting to simulator...\n";
    
    ILCClient client;
    if (!client.connect()) {
        std::cerr << "\nFailed to connect. Make sure the simulator is running.\n";
        return 1;
    }
    
    printHelp();
    
    std::string line;
    while (true) {
        std::cout << "\nilc> ";
        std::getline(std::cin, line);
        
        if (line.empty()) continue;
        
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line == "quit" || line == "exit") {
            std::cout << "Disconnecting...\n";
            break;
        }
        
        if (line == "help" || line == "?") {
            printHelp();
            continue;
        }
        
        // Handle presets
        if (line.substr(0, 6) == "preset") {
            std::istringstream iss(line);
            std::string cmd, preset;
            iss >> cmd >> preset;
            handlePreset(client, preset);
            continue;
        }
        
        // Send command to simulator
        std::string response = client.sendCommand(line);
        std::cout << response;
        
        if (!client.isConnected()) {
            std::cout << "Connection lost. Exiting...\n";
            break;
        }
    }
    
    return 0;
}
