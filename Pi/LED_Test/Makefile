# Makefile to compile LinearAdv_Test.cpp into test.exe with debugging

# Compiler
CXX = g++

# Source and target
SRC = main.cpp
TARGET = LED_test

# Compiler flags
CXXFLAGS = -g -Wall -O0

# Default target
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

# Clean up build artifacts
clean:
	rm -f $(TARGET)