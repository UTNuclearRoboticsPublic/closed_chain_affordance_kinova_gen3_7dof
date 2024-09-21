/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See the examples provided
//    in the code for various affordance and motion types and how to define these inputs.
// 2. The main function has comment blocks for the following use cases. Uncomment as needed.
//    - Basic Use Case: Plan, visualize, and execute a joint trajectory from the current robot configuration.
//    - Optional Advanced Use Cases:
//    - Optional Use Case 1: Plan, visualize, and execute while tracking the status of planning and execution.
//    - Optional Use Case 2: Plan, visualize, and execute from a desired robot start configuration, or plan and
//      visualize without connecting to a real robot.
// Important Note:
// You can run the example tasks directly on the robot. However, in the basic use case, the framework plans
// from the robot's current configuration, which may not be suitable for these examples. As a result, we
// recommend using Optional Use Case 2 to run the example tasks and visualize them in RVIZ. Case 2 provides
// a predefined start configuration to ensure compatibility with the examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>

// Function to block until the robot completes the planned trajectory
void block_until_trajectory_execution(const std::shared_ptr<cca_ros::Status> &motion_status,
                                      const rclcpp::Logger &logger)
{
    rclcpp::Rate loop_rate(4);
    while (*motion_status != cca_ros::Status::SUCCEEDED)
    {
        if (*motion_status == cca_ros::Status::UNKNOWN)
        {
            RCLCPP_ERROR(logger, "Motion was interrupted mid-execution.");
        }
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<cca_ros::CcaRos>("cca_ros", node_options, false, true);

    // Start spinning the node in a separate thread to enable ROS functionalities like parameter reading and joint
    // states
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    rclcpp::sleep_for(std::chrono::seconds(1)); // Sleep for 1 second to ensure ROS is initialized properly

    /// REQUIRED INPUT: Task description. See repo README.md Task Examples.
    cc_affordance_planner::TaskDescription task_description; /// You must fill this out.
    cc_affordance_planner::PlannerConfig planner_config;     /// Fill this out optionally.
   task_description.trajectory_density = 10;

	// Affordance
	affordance_util::ScrewInfo aff;
	//aff.type = affordance_util::ScrewType::ROTATION; // Possible values are ROTATION, TRANSLATION, SCREW
	//aff.axis = Eigen::Vector3d(1, 0, 0); // Eigen::Vector3d representing the affordance screw axis, [1,0,0] for example
	//aff.location = Eigen::Vector3d(0, 0, 1.0); // Eigen::Vector3d representing the location of the affordance screw axis, [0,0,0] for example
	aff.type = affordance_util::ScrewType::TRANSLATION; // Possible values are ROTATION, TRANSLATION, SCREW
	aff.axis = Eigen::Vector3d(0, 0, 1); // Eigen::Vector3d representing the affordance screw axis, [1,0,0] for example
	aff.location = Eigen::Vector3d(0, 0, 0); // Eigen::Vector3d representing the location of the affordance screw axis, [0,0,0] for example

	task_description.affordance_info = aff;

	// Goals
	//task_description.goal.affordance = 1.57; // Goal for the affordance, 0.4 for instance
	task_description.goal.affordance = 0.05; // Goal for the affordance, 0.4 for instance
	task_description.goal.ee_orientation = Eigen::Vector3d(1e-7, 1e-7, 1e-7); // Goal for the affordance, 0.4 for instance

    /*******************************************/
    // BASIC USE CASE: Plan and execute a joint trajectory for a given task from the current robot configuration
     if (!(node->run_cc_affordance_planner(planner_config, task_description))) 
     { 
         RCLCPP_ERROR(node->get_logger(), "Planning and execution failed"); 
     } 

    /*******************************************/
    // OPTIONAL USE CASE 1: Plan, visualize, and execute while tracking the planning and execution status
    /* auto motion_status = */
    /*     std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN); */
    /* if (!(node->run_cc_affordance_planner(planner_config, task_description, motion_status))) */
    /* { */
    /*     RCLCPP_ERROR(node->get_logger(), "Planning and execution failed"); */
    /*     rclcpp::shutdown(); */
    /*     return -1; */
    /* } */
    /* block_until_trajectory_execution(motion_status, node->get_logger()); */

    /*******************************************/
    // OPTIONAL USE CASE 2: Plan, visualize, and execute from a desired start robot configuration
    // cca_ros::KinematicState start_config;
    // start_config.robot = (Eigen::VectorXd(6) << 0.0, -1.09419, 2.2496, -0.567882, -0.796551,
    //                       0.396139)
    //                          .finished(); // Example robot configuration for planning and visualization
    // start_config.gripper = 0;
    /* const Eigen::VectorXd STOW_CONFIG = */
    /*     (Eigen::VectorXd(6) << 0.015582084655761719, -3.13411283493042, 3.1333792209625244, 1.5574960708618164, */
    /*      -0.0033998489379882812, -1.571157455444336) */
    /*         .finished(); */
    /* const Eigen::VectorXd robot_start_config = STOW_CONFIG; */
    // auto motion_status = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);

    // if (!(node->run_cc_affordance_planner(planner_config, task_description, motion_status, start_config)))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning and execution failed");
    //     rclcpp::shutdown();
    //     return -1;
    // }

    // block_until_trajectory_execution(motion_status, node->get_logger());

    rclcpp::shutdown();
    return 0;
}
