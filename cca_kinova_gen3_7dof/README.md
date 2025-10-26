# Usage 
## Planning Request
We use a **request–response model** to communicate with the planner.  
A typical request is structured as follows:

```cpp
cca_ros::PlanningRequest req;
req.task_description = // See examples below
req.execute_trajectory = true; // Set to true to execute the trajectory on the robot (default: false)
req.time_step = // Time interval (in seconds) between consecutive trajectory points
```
#### Optional / Advanced Settings
These are rarely needed in typical usage:
```cpp
req.planner_config = // Custom planner configuration (default settings are usually sufficient). See more info below.
req.start_state = // Specify a starting joint configuration (default: current robot state)
```

## Task Description

Broadly, CCA provides four planning types:

1. **Affordance Planning**: Plan tasks in terms of elementary motions—linear, rotational, or screw. *Affordance* describes the type of motion that an object offers as a task. For instance, a valve "affords" turning, which could be modeled as rotational motion.
2. **Cartesian Goal Planning**: Plan tasks to reach a specified Cartesian goal pose.
3. **In-Place EE Orientation Control**: Plan to adjust the end effector (EE) orientation about a specific axis without translational movement.
4. **Approach Planning**: Given an affordance definition and a reference pose, plan to a pose along the affordance path that achieves a specified affordance goal. For example, move from an arbitrary current configuration to a pose that corresponds to 90 degrees along a rotational path defined by the affordance.

Below, we provide task examples for each of these categories. 

## Defining tasks in Robot Reference Frame
In the following examples, all vectors are defined in the reference frame specified in the `cca_<robot>_description.yaml` or `cca_<robot>_urdf.yaml`.

### Affordance Tasks

##### Translation
We want to move 0.8m along the z-axis

```cpp
// Task instantiation
cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::AFFORDANCE);

// Affordance info
task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
task_description.affordance_info.location = Eigen::Vector3d::Zero();

// Goals
task_description.goal.affordance = 0.8;
// Optional: Constrain the EE to a particular desired orientation along the path
task_description.goal.ee_orientation = Eigen::Vector3d(M_PI/12, M_PI/8, M_PI/12); // as EE-frame rpy
```

##### Rotation
We want to rotate about the z-axis by 270degrees

```cpp
// Task instantiation
cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::AFFORDANCE);

// Affordance info
task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
task_description.affordance_info.location = Eigen::Vector3d(0.0, 0.0, 0.8);

// Goals
task_description.goal.affordance = 3.0 * M_PI / 2.0;
```

##### Screw Motion
We want to perform a screw motion about the negative y-axis by 270degrees at a 0.1m/rad pitch

```cpp
// Task instantiation
cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::AFFORDANCE);

// Affordance info
task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
task_description.affordance_info.axis = Eigen::Vector3d(0, -1, 0);
task_description.affordance_info.location = Eigen::Vector3d(-0.4, 0, 0.5);
task_description.affordance_info.pitch = 0.1;

// Goals
task_description.goal.affordance = 3.0 * M_PI / 2.0;
```

### In-Place End-Effector Orientation Control Task
While keeping the EE position fixed, we want to change its orientation about the x-axis by 90degrees

```cpp
// Task instatiation
cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

// Affordance info
task_description.affordance_info.axis = Eigen::Vector3d(1, 0, 0);

// Goal
task_description.goal.affordance = M_PI / 2.0;
```


### Cartesian Goal Planning Task
We want to plan to a desired cartesian goal

```cpp
// Task instatiation
 cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::CARTESIAN_GOAL);

// Goal
task_description.goal.canonical_pose = Eigen::Matrix4d::Identity(); // as a 4x4 Homogeneous Transformation Matrix
task_description.goal.canonical_pose.block<3, 1>(0, 3) =
    (Eigen::Vector3d() << 0.70932, 0.000336774, -0.017661).finished(); // we specify position, but leave orientation as identity for this example.
```

### Approach Motion Planning Task
We want to move from the current configuration to a pose along the affordance path, positioned 90 degrees from the specified reference (canonical) pose.

```cpp
// Task instatiation
task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);

// Affordance info
task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
task_description.affordance_info.axis = Eigen::Vector3d(1, 0, 0);
task_description.affordance_info.location = Eigen::Vector3d::Zero();

// Goal
task_description.goal.affordance = M_PI / 2.0; // Set desired goal for the affordance
task_description.goal.canonical_pose = Eigen::Matrix4d::Identity();
task_description.goal.canonical_pose.block<3, 1>(0, 3) = Eigen::Vector3d(0.3, 0.3, 0.5);
```

## Defining Tasks in Relation to a Frame of Choice
In the following examples, we specify the task in different frames that can be looked up in the TF tree.

### Affordance Planning Task
We want to do a 90-degree rotation about the z axis of the EE frame
```cpp
// Affordance info from -- Gets axis and location 
task_description.affordance_info.from.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
task_description.affordance_info.from.frame_name = "ee_frame"; // Specify a valid frame name that appears in the TF tree
task_description.affordance_info.from.axis_in_final_pose = affordance_util::axis_to_vec(affordance_util::Axis::Z); // Choose z-axis of that frame

// Specify type
task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;

// Goal
task_description.goal.affordance = M_PI / 2.0; // Set desired goal for the affordance
```

### Cartesian Goal Planning Task
We want to do a 90-degree rotation about the z axis of some frame B defined in relation to some frame A
```cpp
// Affordance info from -- Gets axis and location 
task_description.affordance_info.from.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
task_description.affordance_info.from.frame_name = "frame_a"; // Specify a valid frame name that appears in the TF tree

// Define the transform from frame_a to frame_b -- For simple example, just 10cm along the x-axis.
Eigen::Isometry3d T_a_to_b = Eigen::Isometry3d::Identity();
T_a_to_b.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
task_description.affordance_info.from.post_transform = T_a_to_b.matrix(); 

task_description.affordance_info.from.axis_in_final_pose = affordance_util::axis_to_vec(affordance_util::Axis::Z); // Choose z-axis of frame b

// Specify type
task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;

// Goal
task_description.goal.affordance = M_PI / 2.0; // Set desired goal for the affordance
```

We want to plan to a cartesian pose offset from some frame
```cpp
// Task instatiation
cc_affordance_planner::TaskDescription task_description(cc_affordance_planner::PlanningType::CARTESIAN_GOAL);

task_description.canonical_pose_from.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
task_description.canonical_pose_from.frame_name = "frame_a"; // Specify a valid frame name that appears in the TF tree

// Define the transform from frame_a to frame_b -- 
Eigen::Isometry3d T_a_to_b = Eigen::Isometry3d::Identity();
T_a_to_b.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix(); // 90-deg rotation about the x-axis
T_a_to_b.translation() = Eigen::Vector3d(0.1, 0.0, 0.0); // 10cm along the x-axis
task_description.canonical_pose_from.post_transform = T_a_to_b.matrix(); 
```

## Task Description Settings
Occasionally, you may need to consider the following settings for the task:
```cpp
task_description.trajectory_density = 50; // Require the joint trajectory to have 50 points. By default this is 10
```

## Planner Configuration
And seldom, you may need to touch planner settings
```cpp
cc_affordance_planner::PlannerConfig plannerConfig;
plannerConfig.ik_max_itr = 1000; // Default is 200. Raise this if the planner seems not to solve joint trajectories you think are feasible.
plannerConfig.accuracy = 1.0/100; // Accuracy of the planner, 1% for example. Default is 10%
```
We leave the other settings to the advanced reader to persue through the CCA library documentation
