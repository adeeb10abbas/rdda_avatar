# RDDA Interface for ROS2

This repository contains the ROS2 interface for grippers and gloves, utilizing the RDDA Control Library. It's designed to work with ROS2 Humble on Ubuntu 20.04.

## Installation

This package has been tested on **Ubuntu 20.04** with ROS2 Humble.

1. Install the [RDDA Control Library](https://github.com/RoboticsCollaborative/RDDA) in your workspace:

```git clone -b avatar_final https://github.com/RoboticsCollaborative/RDDA.git```


2. Install this package in your workspace:
```git clone https://github.com/adeeb10abbas/rdda_avatar.git```

3. Build your workspace:
```colcon build```


## Requirements

1. [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. [RDDA Control Library](https://github.com/RoboticsCollaborative/RDDA)

## Features

- Synchronization of joint states and command references between high-level ROS nodes and low-level controller.
- Dynamic parameter tuning (e.g., max velocity/torque and stiffness).
- Data collection and visualization capabilities.

### Joint Control

Subscribe to the *"rdda_interface/joint_states"* topic to obtain the following joint states:
- `position`: Joint angles w.r.t. motor coordinates (radians).
- `velocity`: Joint velocities w.r.t. motor coordinates (radians/second).
- `effort`: External torque/force (Nm).

Publish joint reference positions to the *"rdda_interface/joint_cmds"* topic:
- `positions`: Joint reference positions (radians).

### Parameter Tuning

Use ROS2 services to adjust dynamic parameters at runtime:
- Velocity saturation: `/rdda_interface/set_max_vel` (radians/second)
- Effort/torque saturation: `/rdda_interface/set_max_eff` (Nm)
- Stiffness: `/rdda_interface/set_stiff` (Nm/rad)

### Data Collection & Visualization

Monitor streaming data using tools like [PlotJuggler](https://github.com/facontidavide/PlotJuggler).
