# Build the standalone AMR sensor simulation (no ROS 2 required).
# The ROS 2 node (src/amr_sensor_node.cpp) is built separately with
# colcon/ament in a ROS 2 workspace -- see the header of that file.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2

BIN := amr_sensor_sim
SRC := src/amr_sensor_sim.cpp

.PHONY: all run clean

all: $(BIN)

$(BIN): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(BIN) $(SRC)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
