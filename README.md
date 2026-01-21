# Autonomous Driving Agent in CARLA

This repository contains a modular autonomous driving agent designed for simulation in the **CARLA** autonomous driving simulator. The system integrates perception, planning, decision-making, and control to enable autonomous navigation in dynamic traffic environments.

The project is structured to allow easy experimentation with different planners, controllers, and decision logic while maintaining a clean separation of concerns.

---

## Key Features

- Modular autonomous driving pipeline  
- Lane following and road following behavior  
- Obstacle and traffic spawning for scenario testing  
- B-spline based trajectory planning  
- Radar-based perception processing  
- Rule-based or heuristic decision making  
- Path tracking and low-level vehicle control  
- CARLA simulator interface abstraction  

---

## System Architecture

The system follows a standard autonomous driving stack:

1. **Perception**
   - Radar processing  
   - Environment state extraction  

2. **Planning**
   - Lane offset planning  
   - B-spline trajectory generation  
   - Road and path following  

3. **Decision Making**
   - High-level driving logic  
   - Behavior selection  

4. **Control**
   - Low-level vehicle control (throttle, brake, steering)  

5. **Simulation Interface**
   - CARLA API integration  
   - Traffic and obstacle spawning  

---


## Repository Structure
```text
├── config.py                     # Global configuration parameters
├── main.py                       # Main entry point for running the agent
│
├── carla_interface.py            # CARLA simulator connection & API wrapper
│
├── simple_agent.py               # High-level autonomous agent logic
├── decision.py                   # Behavioral decision-making module
│
├── road_follower.py              # Road following logic
├── path_follower.py              # Path tracking and following controller
│
├── lane_offset_planner.py        # Lane offset and lateral planning
├── bspline_planner.py            # B-spline based trajectory planner
│
├── controller.py                 # Low-level vehicle control (PID / control laws)
│
├── radar_processor.py            # Radar sensor data processing
│
├── traffic_spawner.py            # Traffic actor spawning in CARLA
├── obstacle_spawner.py           # Obstacle spawning for scenario testing
│
├── utils.py                      # Utility and helper functions
│
└── all_code.txt                  # Aggregated code snapshot (for reference)
```
---
## Installation & Setup

### Prerequisites

- Python 3.7+
- CARLA Simulator (compatible version)
- CARLA Python API
- NumPy, SciPy, and other scientific Python dependencies

### Example Setup

```bash
pip install numpy scipy
```
---
## Running the Project

### Start the CARLA Simulator

```bash
./CarlaUE4.sh
```

### Run the Autonomous Agent

```bash
python main.py
```
---

## Core Modules Overview

#### `main.py`

Entry point that initializes CARLA, sets up sensors, spawns actors, and starts the autonomous agent loop.

#### `simple_agent.py`

Implements the main autonomous agent logic, integrating perception, planning, and control.

#### `decision.py`

Contains high-level driving behavior logic such as:

- Lane following  
- Obstacle response  
- Traffic-aware behavior adjustments  

#### `bspline_planner.py`

Generates smooth trajectories using B-spline curves for comfortable and dynamically feasible vehicle motion.

#### `lane_offset_planner.py`

Handles lateral planning within a lane for:

- Obstacle avoidance  
- Lane centering  
- Tactical lateral positioning  

#### `controller.py`

Low-level controller converting desired trajectory and speed into:

- Throttle  
- Brake  
- Steering commands  

#### `radar_processor.py`

Processes radar sensor data to extract obstacle position, distance, and relative velocity information.

#### `traffic_spawner.py` & `obstacle_spawner.py`

Utilities for creating dynamic and static scenarios in CARLA for testing and validation.

---

## Configuration

Global parameters are defined in:

```text
config.py
```

This includes:

- Vehicle parameters
- Controller gains
- Planner tuning parameters
- Simulation settings
