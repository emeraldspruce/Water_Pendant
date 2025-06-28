# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -g -fsanitize=undefined,address -fno-omit-frame-pointer -Wall -Wextra

# Source files
SRCS = test.cpp sim_FLIP.cpp

# Output binary
TARGET = test

# Default build rule
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

# Clean rule
clean:
	rm -f $(TARGET)
