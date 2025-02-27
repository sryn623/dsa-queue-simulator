# TRAFFIC JUNCTION SIMULATOR

🚦 **An Intelligent, Queue-Based Solution for Optimized Traffic Flow** 🚦

## Introduction
Ever wondered how smart traffic systems handle busy intersections? This project simulates an advanced traffic management system using data structures, ensuring smooth and efficient movement through a multi-lane junction.

---
## Features

### 🔄 Smart Queue-Based Management
- Utilizes FIFO (First In, First Out) queue structures for efficient traffic flow.
- Implements a priority queue system to handle congestion effectively.

### 🚦 Adaptive Signal Timing
- Signals dynamically adjust their duration based on real-time vehicle load.
- Prevents excessive wait times and optimizes lane utilization.

### 🚗 Advanced Lane System
- **Standard Lanes** – Follow regular traffic signals.
- **Priority Lane (A2)** – Gains higher priority when congestion exceeds 10 vehicles; reverts when count drops below 5.
- **Express Lane (L3)** – Allows continuous left turns, bypassing traffic signals.

### 🔍 Realistic Traffic Dynamics
- Vehicles follow natural movement patterns.
- Intelligent routing ensures logical intersection navigation.

---
## Technical Overview

### 🛠 Core Data Structures
- **Base Queue System** – Manages vehicle sequencing.
- **Enhanced Priority Queue** – Handles dynamic lane prioritization.
- **Navigation Framework** – Directs vehicle flow through the intersection.

### 📈 Algorithmic Logic
#### Signal Timing Formula
- `Total Signal Time = |V| * t`
- `|V|` = Weighted average of waiting vehicles (`(1/n) * Σ|Li|`)
- `n` = Total number of standard lanes
- `t` = Processing interval per vehicle (2 sec)

#### Adaptive Lane Prioritization
- Lane **A2** gains priority when vehicles >10; returns to normal once vehicles <5.
- Lane **L3** operates in continuous flow mode for left turns, unaffected by signals.

---
## System Requirements

- **C++17 or later** (GCC 8+, Clang 7+, MSVC 19.14+)
- **CMake 3.15+**
- **SDL3 library** (for visualization)

---
## Setup & Installation

### Installing SDL3

#### Linux
```bash
sudo apt-get update
sudo apt-get install build-essential git cmake
git clone https://github.com/libsdl-org/SDL.git -b SDL3
cd SDL && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

#### macOS
```bash
brew install cmake git
git clone https://github.com/libsdl-org/SDL.git -b SDL3
cd SDL && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
sudo make install
```

#### Windows
```powershell
git clone https://github.com/libsdl-org/SDL.git -b SDL3
cd SDL && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
cmake --install . --config Release
```

---
## Build & Execution

### Building the Simulator

#### Linux/macOS
```bash
git clone https://github.com/sarbeshkc/dsa-queue-simulator.git
cd dsa-queue-simulator && mkdir build && cd build
cmake ..
make -j$(nproc)
```

#### Windows
```powershell
git clone https://github.com/sarbeshkc/dsa-queue-simulator.git
cd dsa-queue-simulator && mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

### Running the Simulator

1. Open two terminal windows.
2. In the first terminal, run the traffic generator:
```bash
./bin/traffic_generator # (Windows: .\bin\Release\traffic_generator.exe)
```
3. In the second terminal, launch the simulator:
```bash
./bin/simulator # (Windows: .\bin\Release\simulator.exe)
```

---
## Project Structure
```
dsa-queue-simulator/
├── CMakeLists.txt
├── include/
│   ├── core/ (Simulation Components)
│   │   ├── Lane.h | TrafficLight.h | Vehicle.h
│   ├── managers/ (Control Systems)
│   │   ├── FileHandler.h | TrafficManager.h
│   ├── utils/ (Utilities)
│   │   ├── DebugLogger.h | Queue.h | PriorityQueue.h
│   ├── visualization/ (Graphics)
│   │   ├── Renderer.h
├── src/ (Implementation)
│   ├── core/ | managers/ | utils/ | visualization/
│   ├── main.cpp
│   ├── traffic_generator.cpp
```

---
## Academic Relevance

📌 **COMP202 Assignment #1 Compliance:**
- **Linear Data Structures** – FIFO queue & priority queue implementation.
- **Traffic Scenarios** – Handles both standard and priority-based traffic control.
- **Lane Functionality** – Implements regulated, priority, and continuous-flow lanes.

---
## Contribute & Explore
🚦 **Experience Intelligent Traffic Flow Like Never Before!** 🚦

🔗 [GitHub Repository](https://github.com/sarbeshkc/dsa-queue-simulator)

🎯 *Optimize urban mobility with the power of data structures!*

