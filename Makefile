# PathPlanning Makefile
# Alternative build system to CMake

# Project settings
PROJECT_NAME = path_planning
VERSION = 1.0.0

# Directories
SRC_DIR = src
VIEWER_SRC_DIR = $(SRC_DIR)/viewer
MAP_SRC_DIR = $(SRC_DIR)/map
PATH_SRC_DIR = $(SRC_DIR)/path
AGENT_SRC_DIR = $(SRC_DIR)/agent
THIRD_PARTY_DIR = third_party
INCLUDE_DIR = include
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj
BIN_DIR = $(BUILD_DIR)/bin

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic
INCLUDES = -I$(INCLUDE_DIR) -I$(THIRD_PARTY_DIR)/glad/include -I/usr/include/glm
LIBS = -lglfw -lGL -ldl

# Build type (can be overridden: make BUILD_TYPE=Debug)
BUILD_TYPE ?= Release
ifeq ($(BUILD_TYPE),Debug)
    CXXFLAGS += -g -O0 -DDEBUG
else
    CXXFLAGS += -O3 -DNDEBUG
endif

# Source files
VIEWER_SOURCES = $(wildcard $(VIEWER_SRC_DIR)/*.cpp)
MAP_SOURCES = $(wildcard $(MAP_SRC_DIR)/*.cpp)
PATH_SOURCES = $(wildcard $(PATH_SRC_DIR)/*.cpp)
AGENT_SOURCES = $(wildcard $(AGENT_SRC_DIR)/*.cpp)
GLAD_SOURCE = $(THIRD_PARTY_DIR)/glad/gl.c
MAIN_SOURCE = $(SRC_DIR)/main.cpp

# Object files
VIEWER_OBJECTS = $(VIEWER_SOURCES:$(VIEWER_SRC_DIR)/%.cpp=$(OBJ_DIR)/viewer/%.o)
MAP_OBJECTS = $(MAP_SOURCES:$(MAP_SRC_DIR)/%.cpp=$(OBJ_DIR)/map/%.o)
PATH_OBJECTS = $(PATH_SOURCES:$(PATH_SRC_DIR)/%.cpp=$(OBJ_DIR)/path/%.o)
AGENT_OBJECTS = $(AGENT_SOURCES:$(AGENT_SRC_DIR)/%.cpp=$(OBJ_DIR)/agent/%.o)
GLAD_OBJECT = $(OBJ_DIR)/gl.o
MAIN_OBJECT = $(OBJ_DIR)/main.o
ALL_OBJECTS = $(VIEWER_OBJECTS) $(MAP_OBJECTS) $(PATH_OBJECTS) $(AGENT_OBJECTS) $(GLAD_OBJECT) $(MAIN_OBJECT)

# Target executable
TARGET = $(BIN_DIR)/$(PROJECT_NAME)

# Default target
.PHONY: all
all: $(TARGET)

# Create directories
$(OBJ_DIR)/viewer:
	@mkdir -p $(OBJ_DIR)/viewer

$(OBJ_DIR)/map:
	@mkdir -p $(OBJ_DIR)/map

$(OBJ_DIR)/path:
	@mkdir -p $(OBJ_DIR)/path

$(OBJ_DIR)/agent:
	@mkdir -p $(OBJ_DIR)/agent

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

# Build target executable
$(TARGET): $(ALL_OBJECTS) | $(BIN_DIR)
	@echo "Linking $(PROJECT_NAME)..."
	$(CXX) $(ALL_OBJECTS) -o $@ $(LIBS)
	@echo "Build complete: $@"

# Build core object files
$(OBJ_DIR)/viewer/%.o: $(VIEWER_SRC_DIR)/%.cpp | $(OBJ_DIR)/viewer
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/map/%.o: $(MAP_SRC_DIR)/%.cpp | $(OBJ_DIR)/map
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/path/%.o: $(PATH_SRC_DIR)/%.cpp | $(OBJ_DIR)/path
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR)/agent/%.o: $(AGENT_SRC_DIR)/%.cpp | $(OBJ_DIR)/agent
	@echo "Compiling $<..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Build GLAD object file
$(OBJ_DIR)/gl.o: $(GLAD_SOURCE) | $(OBJ_DIR)
	@echo "Compiling GLAD..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Build main object file
$(OBJ_DIR)/main.o: $(MAIN_SOURCE) | $(OBJ_DIR)
	@echo "Compiling main..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(BUILD_DIR)

# Clean and rebuild
.PHONY: rebuild
rebuild: clean all

# Install target (optional)
.PHONY: install
install: $(TARGET)
	@echo "Installing $(PROJECT_NAME) to /usr/local/bin..."
	sudo cp $(TARGET) /usr/local/bin/

# Uninstall target (optional)
.PHONY: uninstall
uninstall:
	@echo "Removing $(PROJECT_NAME) from /usr/local/bin..."
	sudo rm -f /usr/local/bin/$(PROJECT_NAME)

# Run the program
.PHONY: run
run: $(TARGET)
	@echo "Running $(PROJECT_NAME)..."
	./$(TARGET)

# Show help
.PHONY: help
help:
	@echo "PathPlanning Makefile"
	@echo "=================="
	@echo "Targets:"
	@echo "  all      - Build the project (default)"
	@echo "  clean    - Remove build artifacts"
	@echo "  rebuild  - Clean and build"
	@echo "  install  - Install to /usr/local/bin"
	@echo "  uninstall- Remove from /usr/local/bin"
	@echo "  run      - Build and run the program"
	@echo "  help     - Show this help message"
	@echo ""
	@echo "Variables:"
	@echo "  BUILD_TYPE - Debug or Release (default: Release)"
	@echo "  CXX        - C++ compiler (default: g++)"
	@echo ""
	@echo "Examples:"
	@echo "  make                    # Build in Release mode"
	@echo "  make BUILD_TYPE=Debug   # Build in Debug mode"
	@echo "  make run               # Build and run"
	@echo "  make clean             # Clean build artifacts"

# Show project info
.PHONY: info
info:
	@echo "Project: $(PROJECT_NAME)"
	@echo "Version: $(VERSION)"
	@echo "Build Type: $(BUILD_TYPE)"
	@echo "Compiler: $(CXX)"
	@echo "Flags: $(CXXFLAGS)"
	@echo "Includes: $(INCLUDES)"
	@echo "Libraries: $(LIBS)"
	@echo "Target: $(TARGET)"

# Dependencies tracking (basic)

$(OBJ_DIR)/%.d: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MM -MT $(@:.d=.o) $< > $@

$(OBJ_DIR)/viewer/%.d: $(VIEWER_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MM -MT $(@:.d=.o) $< > $@
$(OBJ_DIR)/agent/%.d: $(AGENT_SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -MM -MT $(@:.d=.o) $< > $@