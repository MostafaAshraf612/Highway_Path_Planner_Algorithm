# 🚗 Highway Path Planning Algorithm  
**Udacity Self-Driving Car Nanodegree (v1.0)**  
**Developer:** [Mostafa Ashraf El Sayed](https://www.linkedin.com/in/mostafa-ashraf-612)

![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![Language: C++](https://img.shields.io/badge/Language-C++11-blue.svg)
![Status: Completed](https://img.shields.io/badge/Status-Completed-success.svg)

---

## 📚 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Build & Execution](#build--execution)
- [Algorithm Summary](#algorithm-summary)
- [Decision Logic](#decision-logic)
- [Results & Demonstration](#results--demonstration)
- [License](#license)
- [Contact](#contact)

---

## 📌 Overview

A high-performance **C++ highway path planner** built for real-time autonomous driving in a simulated environment.  
This system generates **safe**, **smooth**, and **efficient trajectories** using localization and sensor fusion data, enabling the vehicle to navigate multi-lane traffic while respecting kinematic and safety constraints.

Developed as part of the **Udacity Self-Driving Car Nanodegree**, the planner integrates prediction, behavior planning, and spline-based trajectory generation for continuous, jerk-limited motion.

---

## ✨ Features

- **Real-Time Planning:** Continuously updates trajectories using live telemetry  
- **Spline-Based Path Generation:** Ensures smooth transitions with minimal jerk and acceleration  
- **Dynamic Behavior Planning:** Determines optimal lane and speed based on traffic context  
- **Sensor Fusion Integration:** Tracks surrounding vehicles for collision-free decisions  
- **Modular Architecture:** Clean separation of localization, prediction, planning, and control components  

---

## 🧠 Architecture

1. **Localization** – Retrieves ego vehicle state (position, orientation, velocity)  
2. **Prediction** – Processes sensor fusion input to detect and forecast nearby vehicle motion  
3. **Behavior Planning** – Chooses target lane and velocity based on traffic conditions  
4. **Trajectory Generation** – Fits a spline to produce a smooth, feasible path  
5. **Control Output** – Sends computed waypoints to the simulator for actuation  

---

## 📁 Repository Structure



```
Highway_Path_Planner/
│
├── src/
│ ├── main.cpp # Main path planning workflow
│ ├── helpers.cpp # Utility functions
│ 
│
├── include/
│ ├── helpers.h
│ └── polynomial_solver.h # Optional polynomial utilities
| └── spline.h # Spline interpolation library
│
├── data/
│ └── highway_map.csv # Map waypoints
│
├── CMakeLists.txt
└── README.md
```
---

## 🛠️ Build & Execution

### 🔧 Requirements

- C++11 or later  
- [uWebSockets](https://github.com/uNetworking/uWebSockets)  
- [Udacity Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)  

### 📦 Steps
---


### 🔧 **__Step 1: Clone the Repository__**

```bash
git clone https://github.com/MostafaAshraf612/Highway_Path_Planner_Algorithm.git
cd Highway_Path_Planner_Algorithm
```
### 🔧 **__Step 2: Build The Project__**
```bash
mkdir build && cd build
cmake ..
make
```
### 🔧 **__Step 3: Run The Planner__**
```bash
./path_planning
```
---
## 📈 **Algorithm Summary**

The planner continuously evaluates the dynamic driving environment to decide whether to **_maintain the current lane_**, **_perform a lane change_**, or **_adjust the vehicle’s speed_**.  
It uses **spline-based trajectory generation** to ensure **smooth**, **jerk-limited motion** while maintaining **comfort and safety**.

---

## 🧭 **Decision Logic**

- **Detect slower vehicles ahead** using sensor fusion data  
- **Evaluate adjacent lanes** for safe lane change opportunities  
- **Execute lane change** if feasible; otherwise, **smoothly decelerate**  
- **Continuously update trajectory** to adapt to real-time traffic and vehicle dynamics  

---

## 🎥 **Results & Demonstration**

The planner successfully maintains safe distances, performs smooth lane changes, and reaches the target speed without exceeding jerk or acceleration limits.

📹 **Demo Video:** [Watch on Google Drive](https://drive.google.com/file/d/1B3jG_mBzdoHw_HwY-1dph1Qs24xOCvsB/view)

---

### ✅ **Performance Metrics**

| 🔍 **Metric**                              | 📊 **Value**              | 📝 **Description**                             |
|--------------------------------------------|----------------------------|------------------------------------------------|
| **Target Speed**                           | 49.5 mph (±0.5 mph)        | Maintains near-limit speed safely              |
| **Maximum Jerk**                           | < 10 m/s³                  | Ensures passenger comfort and stability        |
| **Lane Change Time**                       | < 1 second                 | Fast and smooth transitions between lanes      |
| **Collisions**                             | 0                          | No collisions during validation scenarios      |
| **Path Smoothness**                        | High                       | Spline interpolation minimizes abrupt motion   |

---

## 📄 **License**

This project is released under the **[MIT License](LICENSE)**.

---

## 📬 **Contact**

For technical inquiries or collaboration opportunities:

**Mostafa Ashraf El Sayed**  
🔗 [LinkedIn](https://www.linkedin.com/in/mostafa-ashraf-612)  
💻 [GitHub](https://github.com/MostafaAshraf612)  
📧 [mostafashrafelsayed612@gmail.com](mailto:mostafashrafelsayed612@gmail.com)



