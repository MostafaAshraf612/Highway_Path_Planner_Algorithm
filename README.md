# Highway Path Planner - Udacity Self-Driving Car Nanodegree v1.0

## Project Overview  
This repository contains a path planner algorithm developed as part of the Udacity Self-Driving Car Nanodegree program (version 1.0). The primary objective of this project is to generate a smooth and safe trajectory for an autonomous vehicle traveling on a highway. The path is represented as a series of waypoints that the vehicle must follow, taking into account other traffic and lane constraints.

## Project Demo  
<video width="600" height="400" controls>
  <source src="https://github.com/user-attachments/assets/0010789b-f6c7-47ee-899d-367f57f7bf4f" type="video/mp4">


[Watch the full working demo video here](https://your-video-link)  

*The demo video showcases the vehicle navigating the highway with lane keeping, lane changing, and adaptive speed control.*

## Features
- Generates smooth trajectories using spline interpolation.
- Considers vehicle localization, previous path points, and sensor fusion data to detect nearby vehicles.
- Implements lane keeping and safe lane change maneuvers.
- Maintains velocity control to comply with speed limits and adjust speed based on traffic.
- Integrates with Udacity’s self-driving car simulator for real-time path planning.

## General Code Flow

1. **Initialization**: The program loads the highway map waypoints from a CSV file, including coordinates and directional vectors for all lanes.

2. **WebSocket Communication**: Establishes connection with the Udacity simulator to receive telemetry and sensor fusion data and send trajectory points.

3. **Telemetry Data Processing**: Receives the current vehicle position, orientation, speed, and the unconsumed previous path points.

4. **Sensor Fusion Analysis**: Processes data about all other vehicles on the road to assess their speed and position relative to the ego vehicle in real-time.

5. **Decision Making**:  
   - If the vehicle ahead is too close, the planner evaluates whether a lane change is safe and beneficial by checking adjacent lanes for gaps ahead and behind.  
   - If lane change is unsafe, the vehicle slows down accordingly.  
   - If no obstacle is close, the vehicle accelerates to the target speed (close to speed limit).  

6. **Path Generation Using Splines**:  
   - Using a combination of previous path points and new waypoints spaced ahead on the highway, the planner fits a spline curve to create a smooth trajectory.  
   - Points are generated along this spline spaced to maintain the target speed, then transformed back to global coordinates.

7. **Trajectory Output**: The planned points are sent to the simulator which controls the autonomous vehicle to follow the planned path smoothly.

## Project Structure
- **main.cpp**: Main driver code implementing the complete path planning workflow.  
- **helpers.h / helpers.cpp**: Utility functions for coordinate transforms and waypoint manipulation.  
- **spline.h**: tk spline library used for smooth path generation.  
- **data/highway_map.csv**: Predefined highway waypoints.  
- **polynomial_solver.h**: Polynomial helper functions (optional).

## Getting Started

### Prerequisites  
- Linux or Windows machine with C++11 or later compiler  
- uWebSockets library for WebSocket communication  
- Eigen3 library for linear algebra computations  
- Udacity Self-Driving Car Simulator  

### Installation and Build  
1. Clone the repository  
2. Ensure all dependencies are installed, including uWebSockets and Eigen3  
3. Build the project using your preferred C++ build system (e.g., CMake or Makefiles)  
4. Run the executable, which listens for connections from the Udacity Simulator

### Running the Project  
1. Launch the Udacity Self-Driving Car Simulator and select the highway map  
2. Run the path planner executable  
3. The vehicle autonomously drives on the highway following the generated waypoints

## Contributing  
Contributions and improvements are welcome. Please submit pull requests with clear explanations and ensure code comments maintain clarity and consistency.

## References  
- Udacity Self-Driving Car Nanodegree Program: Path Planning Project  
- uWebSockets documentation  
- Eigen Library  
- Spline Library  

## License  
MIT License.
```
