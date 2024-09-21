# Closed-chain Affordance Planning for NRG kinova_gen3_7dof using ROS2
This repository contains the `cca_kinova_gen3_7dof` package to implement the closed-chain affordance planning framework on the NRG kinova_gen3_7dof robot.

## Build and Install Instructions:
1. Install the closed-chain affordance planning libraries by following instructions from the following repository:
   [Link to instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_ros.git)

2. Clone this repository onto your local machine ROS2 workspace `src` folder:
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
### Notable Dependencies
   `xterm`, install with `sudo apt install xterm`

## Run Instructions:

1. Run the kinova_gen3_7dof driver, or for mock hardware, run the following:
   ```
   ros2 launch kinova_gen3_6dof_robotiq_2f_85_moveit_config robot.launch.py \
     robot_ip:=yyy.yyy.yyy.yyy \
     use_fake_hardware:=true
   ```
2. Optional: If visualizing planned trajectories, `robot_description_semantic` must be loaded onto the parameter server. If the driver launcher does not take care of it, do it separately.


3. Optional: Run the planning visualization server. Then, in RViz, add MarkerArray with publishing topic, `/rviz_visual_tools`.
   ```
   ros2 run cca_ros_viz cca_ros_viz_node
   ```

5. Run the closed-chain affordance planner node for Kinova:
   ```
   ros2 launch cca_kinova_gen3_7dof cca_kinova_gen3_7dof_launch.py
   ```

## Author
Janak Panthi aka Crasun Jans
