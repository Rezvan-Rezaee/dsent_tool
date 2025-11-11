# === Makefile for DSENT Standalone Build ===
# Build all DSENT .cpp and .cc sources recursively

CXX := g++
CXXFLAGS := -O2 -std=c++11 -Wall
TARGET := dsent

# Find all C++ source files recursively (.cpp and .cc)
SRCS := $(shell find . -type f \( -name "*.cpp" -o -name "*.cc" \))
OBJS := $(SRCS:.cpp=.o)
OBJS := $(OBJS:.cc=.o)

# Add all relevant include directories
INCLUDES := \
	-I. \
	-Iutil \
	-Ilibutil \
	-Imodel \
	-Imodel/common \
	-Imodel/electrical \
	-Imodel/network \
	-Imodel/optical \
	-Imodel/optical_graph \
	-Imodel/router \
	-Imodel/std_cells \
	-Imodel/timing_graph \
	-Itech

# Build target
all: $(TARGET)

$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)..."
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp
	@echo "Compiling $< ..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.cc
	@echo "Compiling $< ..."
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
