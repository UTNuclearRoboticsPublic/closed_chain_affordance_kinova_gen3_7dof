# Closed-Chain Affordance Planning for Kinova Gen3 7DoF Arm

## Overview

The `cca_kinova_gen3_7dof` package implements the Closed-Chain Affordance (CCA) planning framework for the Kinova Gen3 7DoF robotic arm using ROS2. This package provides comprehensive tools for joint trajectory planning, visualization, and execution.

## Dependencies

Before installation, ensure you have the following dependencies:

- [CCA Libraries](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)
- [CCA ROS Interface](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)
- [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex): For robot description and optional simulator

## Installation

1. Navigate to your ROS2 workspace and clone the repository:
   ```bash
   cd ~/<ros2_ws_name>/src
   git clone -b main git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git
   ```

2. Build the package and source the workspace:
   ```bash
   cd ~/<ros2_ws_name>
   colcon build --packages-select cca_kinova_gen3_7dof
   source install/setup.bash
   ```

## Usage

### With Physical Robot

1. Launch the Kinova driver:
   ```bash
   ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
     robot_ip:=yyy.yyy.yyy.yyy
   ```
   *Replace `yyy.yyy.yyy.yyy` with your robot's actual IP address*

2. Launch the CCA visualizer with interactive RVIZ plugin:
   ```bash
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_viz.launch.py
   ```
   *This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution*

3. For programmatic task definition, launch the CCA planner which will plan for the tasks defined in `src/cca_kinova_gen3_7dof_node.cpp`:
   ```bash
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof.launch.py
   ```

### Without Physical Robot (Simulation)

1. Launch the fake hardware simulator:
   ```bash
   ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
     robot_ip:=yyy.yyy.yyy.yyy \
     use_fake_hardware:=true
   ```

2. Launch the CCA visualizer with interactive RVIZ plugin:
   ```bash
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_viz.launch.py
   ```
   *This launches both the visualizer and an interactive RVIZ plugin for code-free planning and execution*

3. Run the CCA planner demo which will plan for the tasks defined in `src/demo/cca_kinova_gen3_7dof_demo.cpp`:
   ```bash
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_demo.launch.py
   ```

## Recommendations

- Explore the interactive RVIZ plugin for code-free planning and execution
- Modify demo node tasks to create custom trajectories
- Refer to the package's `README.md` for additional task examples

## Author

**Janak Panthi** (aka Crasun Jans)

## Support

For issues, feature requests, or contributions, please open an issue in the GitHub repository.
