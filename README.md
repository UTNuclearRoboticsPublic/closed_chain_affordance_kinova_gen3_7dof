## Author
Janak Panthi aka Crasun Jans
# Closed-Chain Affordance Planning for the Kinova Gen3 7DoF Arm using ROS2
This repository contains the `cca_kinova_gen3_7dof` package, which implements the closed-chain affordance planning framework on the Kinova Gen3 7DoF arm.

## Dependencies

- **Required**: Robot Description for visualization purposes.
- **Recommended**: Robot driver for testing on a real or simulated robot.

All dependencies can be installed from the official Kinova repository [here](https://github.com/Kinovarobotics/ros2_kortex).

## Build and Install Instructions:
1. Install the closed-chain affordance planning libraries by following instructions from the following repository:
   [Link to instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)

2. Clone this repository onto your machine ROS2 workspace `src` folder:
   ```
   cd ~/<ros2_ws_name>/src
   ```
   ```
   git clone -b main git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git
   ```

3. Build and source the `cca_kinova_gen3_7dof` package:
   ```
   cd ~/<ros2_ws_name>
   ```
   ```
   colcon build --packages-select cca_kinova_gen3_7dof
   ```
   ```
   source install/setup.bash
   ```

## Run Instructions:

### Using with a Physical Robot
To execute CCA-generated trajectories on a real Kinova Gen3 7DoF robot:

1. Run the Kinova driver:
   ```
   ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
     robot_ip:=yyy.yyy.yyy.yyy \
   ```
   Provide your particular robot's IP

2. Launch the CCA visualizer for Kinova:

   ```
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_viz.launch.py
   ```

3. Launch the CCA planner/executor:
   ```
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof.launch.py
   ```
### Using without a Physical Robot
You can plan and visualize trajectories for the Kinova arm using the CCA framework without needing a physical robot. The following demonstration showcases various CCA framework features on the Kinova robot.

1. Launch the CCA-visualizer for Kinova:

   ```
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_viz.launch.py
   ```

2. Launch the CCA planner demo node:
   ```
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_demo.launch.py
   ```

You are encouraged to modify the tasks in the demo node to plan and visualize trajectories tailored to your specific applications. Task examples are also provided in the package README.md. If you want to test with fake hardware, you can run the following driver.

   ```
   ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
     robot_ip:=yyy.yyy.yyy.yyy \
     use_fake_hardware:=true
   ```

## Author
Janak Panthi aka Crasun Jans
