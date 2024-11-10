## Usage - Task Examples

Broadly, CCA provides four core features:

1. **Affordance Planning**: Plan tasks in terms of elementary motions—linear, rotational, or screw. *Affordance* describes the type of motion that an object offers as a task. For instance, a valve "affords" turning, which could be modeled as rotational motion.
2. **Cartesian Goal Planning**: Plan tasks to reach a specified Cartesian goal pose.
3. **In-Place EE Orientation Control**: Plan to adjust the end effector (EE) orientation about a specific axis without translational movement.
4. **Approach Planning**: Given an affordance definition and a reference pose, plan to a pose along the affordance path that achieves a specified affordance goal. For example, move from an arbitrary current configuration to a pose that corresponds to 90 degrees along a rotational path defined by the affordance.

Below, we provide task examples for each of these categories. Note all vectors are defined with respect to a reference frame, which for Spot is the arm base link.

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
task_description.goal.grasp_pose = Eigen::Matrix4d::Identity(); // as a 4x4 Homogeneous Transformation Matrix
task_description.goal.grasp_pose.block<3, 1>(0, 3) =
    (Eigen::Vector3d() << 0.70932, 0.000336774, -0.017661).finished(); // we specify position, but leave orientation as identity for this example.
```

### Approach Motion Task
We want to move from the current configuration to a pose along the affordance path, positioned 90 degrees from the specified reference (grasp) pose.

```cpp
// Task instatiation
task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);

// Affordance info
task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
task_description.affordance_info.axis = Eigen::Vector3d(1, 0, 0);
task_description.affordance_info.location = Eigen::Vector3d::Zero();

// Goal
task_description.goal.affordance = M_PI / 2.0; // Set desired goal for the affordance
task_description.goal.grasp_pose = Eigen::Matrix4d::Identity();
task_description.goal.grasp_pose.block<3, 1>(0, 3) = Eigen::Vector3d(0.3, 0.3, 0.5);
```
