# Hierarchical Adaptive Cruise Control System for Semi-Autonomous Electric Vehicles

## Overview
This project focuses on developing a **Hierarchical Adaptive Cruise Control (ACC) System** specifically for semi-autonomous electric vehicles. The system enhances driving safety and comfort by automatically adjusting the vehicle's speed to maintain a safe distance from other vehicles.

The system uses a two-tiered control architecture:
1. **Upper Controller**: Employs Model Predictive Control (MPC) for longitudinal acceleration management.
2. **Lower Controller**: Focuses on optimal torque distribution and wheel slip management for enhanced stability and traction.

## Key Features
- **Collision Avoidance**: Predicts and mitigates collision risks using Gaussian-based probabilistic modeling.
- **Driving Comfort**: Balances desired speed maintenance with smooth acceleration and deceleration.
- **Traction Optimization**: Distributes torque efficiently to ensure wheel stability across various road conditions.

## System Architecture
### 1. Upper Controller
- **Collision Risk Model**: Utilizes radar, cameras, and GPS data to assess potential collision scenarios.
- **Optimizer**: Calculates optimal longitudinal force to ensure safety and ride comfort.

### 2. Lower Controller
- **Torque Vectoring**: Manages wheel force distribution to meet desired acceleration.
- **Anti-Slip Control**: Prevents wheel slip during sudden maneuvers or challenging road conditions.

### 3. Sensor Integration
- **IMU Radar**: Combines object detection and inertial measurements for precise vehicle dynamics.
- **Camera**: Identifies and tracks objects, traffic signs, and lane markings.
- **GPS**: Provides accurate positioning for navigation and route planning.

## Optimization Algorithm
- **Stage 1**: Predicts desired acceleration using MPC, factoring in Gaussian-modeled collision risks.
- **Stage 2**: Applies torque vectoring to achieve acceleration while managing anti-slip constraints.

## Advantages and Challenges
### Advantages
- Integrates safety and operational constraints directly into optimization.
- Proactively responds to potential risks using predictive models.
- Handles complex dynamics and environmental changes effectively.

### Challenges
- Computationally intensive due to real-time optimization at every time step.
- Requires robust hardware to handle high processing demands.

## Implementation
- **Simulation Environment**: Tested under diverse conditions to validate safety and comfort improvements.
- **Roles**:
  - **Team Leader**: Oversees project progress and compiles final outputs.
  - **Technical Lead**: Develops and tests algorithms.
  - **Documentation Specialist**: Records development processes and prepares presentation materials.

## References
- [Potential Field-Based Hierarchical Adaptive Cruise Control for Semi-Autonomous Electric Vehicles (ResearchGate)](https://www.researchgate.net)
