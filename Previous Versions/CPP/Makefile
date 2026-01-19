# Makefile for ILC Real-Time Tracker

CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
LDFLAGS_SIM = -lX11 -lpthread
LDFLAGS_CLIENT = -lpthread

BUILD_DIR = build
SRC_DIR = .

SIMULATOR = $(BUILD_DIR)/ilc_simulator
CLIENT = $(BUILD_DIR)/ilc_client

.PHONY: all clean run-sim run-client demo help install

# Default target
all: $(SIMULATOR) $(CLIENT)
	@echo ""
	@echo "Build complete!"
	@echo "  Simulator: $(SIMULATOR)"
	@echo "  Client:    $(CLIENT)"
	@echo ""
	@echo "Run 'make run-sim' in one terminal"
	@echo "Run 'make run-client' in another terminal"

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build simulator
$(SIMULATOR): $(SRC_DIR)/simulator.cpp | $(BUILD_DIR)
	@echo "Compiling simulator..."
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS_SIM)
	@echo "✓ Simulator built"

# Build client
$(CLIENT): $(SRC_DIR)/client.cpp | $(BUILD_DIR)
	@echo "Compiling client..."
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS_CLIENT)
	@echo "✓ Client built"

# Run simulator
run-sim: $(SIMULATOR)
	@echo "Starting simulator..."
	./$(SIMULATOR)

# Run client
run-client: $(CLIENT)
	@echo "Starting client..."
	./$(CLIENT)

# Run demo
demo: $(CLIENT)
	@if [ ! -f demo.sh ]; then \
		echo "Error: demo.sh not found"; \
		exit 1; \
	fi
	@chmod +x demo.sh
	@./demo.sh

# Clean build artifacts
clean:
	@echo "Cleaning build directory..."
	rm -rf $(BUILD_DIR)
	@echo "✓ Clean complete"

# Install dependencies (Ubuntu/Debian)
install:
	@echo "Installing dependencies..."
	sudo apt-get update
	sudo apt-get install -y build-essential libx11-dev
	@echo "✓ Dependencies installed"

# Help target
help:
	@echo "ILC Real-Time Tracker - Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make              - Build both simulator and client (default)"
	@echo "  make all          - Same as 'make'"
	@echo "  make run-sim      - Build and run simulator"
	@echo "  make run-client   - Build and run client"
	@echo "  make demo         - Run automated demo (requires running simulator)"
	@echo "  make clean        - Remove build artifacts"
	@echo "  make install      - Install system dependencies (Ubuntu/Debian)"
	@echo "  make help         - Show this help"
	@echo ""
	@echo "Quick start:"
	@echo "  Terminal 1: make run-sim"
	@echo "  Terminal 2: make run-client"
	@echo ""
	@echo "Development workflow:"
	@echo "  1. Edit simulator.cpp or client.cpp"
	@echo "  2. make clean && make"
	@echo "  3. Test changes"
	@echo ""
