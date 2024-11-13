# Drone Navigation with Extended Kalman Filter and ROS2

## Overview
This MATLAB script implements drone navigation using an Extended Kalman Filter (EKF) with ROS2 integration for position tracking and estimation.

## Features
- ROS2 integration for position publishing
- Extended Kalman Filter implementation
- 3D trajectory tracking
- State estimation and visualization

## Components
### ROS2 Setup
- Node creation: "/matlab_node"
- Publisher: '/drone/position'
- Message type: geometry_msgs/Point

### Parameters
- Mass: 1.5 kg
- Force: [0, 0, 14.7]
- Initial position: [10, 5, 100]
- Initial velocity: [1, 1, 0]
- Time step: 0.1 seconds

### Functions
- `stateTransitionFcn`: State prediction
- `measurementFcn`: Measurement model
- `getROSPosition`: ROS position retrieval

## Visualization
- 3D trajectory plot
- Position plots (X, Y, Z)
- Velocity plots (X, Y, Z)

## Usage
1. Ensure ROS2 is properly configured
2. Run the script
3. Monitor trajectory and state estimation plots

## Dependencies
- MATLAB ROS2 Toolbox
- Extended Kalman Filter Toolbox

## Outputs
![D1](https://github.com/user-attachments/assets/8a0ca94f-3102-442c-a010-f0a6b1bbc2d1)
![D2](https://github.com/user-attachments/assets/8d2f1790-e3f3-4664-abda-6c4dfadd6ab9)
![D4](https://github.com/user-attachments/assets/b742f1e8-5990-4b17-97f6-38999a8fd8bf)
