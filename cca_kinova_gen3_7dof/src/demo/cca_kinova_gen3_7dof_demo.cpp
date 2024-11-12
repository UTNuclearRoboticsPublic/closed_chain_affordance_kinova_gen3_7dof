/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See repo README.md Task
// Examples section for task-description examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>

class CcaKinova : public cca_ros::CcaRos
{
  public:
    explicit CcaKinova(const std::string &node_name, const rclcpp::NodeOptions &node_options, bool visualize_trajectory,
                       bool execute_trajectory)
        : cca_ros::CcaRos(node_name, node_options, visualize_trajectory, execute_trajectory)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cc_affordance_planner::PlannerConfig &planner_config,
             const cc_affordance_planner::TaskDescription &task_description,
             const cca_ros::KinematicState &start_config = cca_ros::KinematicState())
    {
        includes_gripper_goal_ = !std::isnan(task_description.goal.gripper);
        motion_status_ = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);

        return this->run_cc_affordance_planner(planner_config, task_description, motion_status_, start_config);
    }

    // Function overload to plan multiple tasks at once
    bool run(const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
             const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions,
             const cca_ros::KinematicState &start_config = cca_ros::KinematicState())
    {
        includes_gripper_goal_ = !std::isnan(task_descriptions[0].goal.gripper);
        motion_status_ = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);

        return this->run_cc_affordance_planner(planner_configs, task_descriptions, motion_status_, start_config);
    }

    // Function to block until the robot completes the planned trajectory
    void block_until_trajectory_execution()
    {
        rclcpp::Rate loop_rate(4);
        auto start_time = std::chrono::steady_clock::now();

        while (*motion_status_ != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status_ == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_logger(), "Motion was interrupted mid-execution.");
                auto current_time = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60)
                {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for motion to complete.");
                    return;
                }
            }
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Exiting due to ROS signal");
                return;
            }
            loop_rate.sleep();
        }
        if (includes_gripper_goal_)
        {
            // Perform any necessary cleanup
            this->cleanup_between_calls();
        }
    }

  private:
    std::shared_ptr<cca_ros::Status> motion_status_;
    bool includes_gripper_goal_ = false;
};

// Demo motions in order
enum DemoMotion
{
    ROLL_FORWARD,
    ROLL_BACKWARD,
    PITCH_FORWARD,
    PITCH_BACKWARD,
    YAW_FORWARD,
    YAW_BACKWARD,
    APPROACH,
    TRANSLATION,
    ROTATION,
    SCREW,
    CARTESIAN_GOAL
};

// Task descriptions for the demo motions
std::pair<cc_affordance_planner::PlannerConfig, cc_affordance_planner::TaskDescription> get_demo_description(
    const DemoMotion &demo_motion)
{
    // Default planner info
    cc_affordance_planner::PlannerConfig planner_config;
    cc_affordance_planner::TaskDescription task_description;

    // The following demo motions happen in order. Read the headline comment for each demo motion to understand what
    // that task does.
    switch (demo_motion)
    {

        // Do an in-place roll adjustment by 90degrees
    case DemoMotion::ROLL_FORWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(1, 0, 0);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Do an in-place roll adjustment by -90degrees
    case DemoMotion::ROLL_BACKWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(-1, 0, 0);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Do an in-place pitch adjustment by 90degrees
    case DemoMotion::PITCH_FORWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(0, 1, 0);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Do an in-place pitch adjustment by -90degrees
    case DemoMotion::PITCH_BACKWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(0, -1, 0);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Do an in-place yaw adjustment by 90degrees
    case DemoMotion::YAW_FORWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Do an in-place yaw adjustment by -90degrees
    case DemoMotion::YAW_BACKWARD:
        task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);

        // Affordance info
        task_description.affordance_info.axis = Eigen::Vector3d(0, 0, -1);

        // Goals
        task_description.goal.affordance = M_PI / 2.0;

        break;

        // Move from the current configuration to a pose along the affordance path,
        // positioned 90 degrees from the specified reference (grasp) pose.
    case DemoMotion::APPROACH:
        task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);

        // Affordance info
        task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
        task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
        task_description.affordance_info.location = Eigen::Vector3d::Zero();
        task_description.trajectory_density = 5;
        // task_description.vir_screw_order = affordance_util::VirtualScrewOrder::XYZ;

        // Goals
        task_description.goal.affordance = 0.5; // Set desired goal for the affordance
        task_description.goal.grasp_pose = Eigen::Matrix4d::Identity();
        task_description.goal.grasp_pose.block<3, 1>(0, 3) = Eigen::Vector3d(-0.3, -0.3, 0.5);
        task_description.force_correction = (Eigen::VectorXd(6) << 0.5, 0.0, 0, 0.0, 0.0, 0.0).finished();
        break;

        // Do a linear motion along the z axis while constraining the EE yaw to a desired value
    case DemoMotion::TRANSLATION:

        task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

        // Affordance info
        task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
        task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
        task_description.affordance_info.location = Eigen::Vector3d::Zero();

        // Goals
        task_description.goal.affordance = 0.8;
        task_description.goal.ee_orientation =
            (Eigen::VectorXd(1) << M_PI / 12.0).finished(); // Optionally, specify EE orientation constraint as rpy. In
                                                            // this example we are only constraining yaw.

        break;

        // Do a rotational motion about the y axis.
    case DemoMotion::ROTATION:
        task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
        task_description.trajectory_density = 20;

        // Affordance info
        task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
        task_description.affordance_info.axis = Eigen::Vector3d(0, 1, 0);
        task_description.affordance_info.location = Eigen::Vector3d(0.0, 0.0, 0.8);

        // Goals
        task_description.goal.affordance = 3.0 * M_PI / 2.0;

        break;

        // Do a screw motion about the -z axis.
    case DemoMotion::SCREW:
        task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
        task_description.trajectory_density = 20;

        // Affordance info
        task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
        task_description.affordance_info.axis = Eigen::Vector3d(0, 0, -1);
        task_description.affordance_info.location = Eigen::Vector3d::Zero();
        task_description.affordance_info.pitch = 0.1;

        // Goals
        task_description.goal.affordance = M_PI;

        break;

        // Go back to the start configuration of the demo using cartesian planning
    case DemoMotion::CARTESIAN_GOAL:
        task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::CARTESIAN_GOAL);

        // Goal
        task_description.goal.grasp_pose =
            (Eigen::Matrix4d() << 0.001471, -0.000767, 0.999999, 0.649786, 0.999999, -0.000341, -0.001471, 0.000102,
             0.000343, 1.000000, 0.000767, 0.434399, 0.000000, 0.000000, 0.000000, 1.000000)
                .finished();
        break;
    }

    return std::make_pair(planner_config, task_description);
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcaKinova>("cca_ros", node_options, true, false);
    RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

    // Spin the node so joint states can be read
    std::thread spinner_thread([node]() { rclcpp::spin(node); });

    /// REQUIRED INPUT: Task description. For demo purposes, here, we plan 11 different tasks at once that showcase
    /// various features of the planning framework. For practical purposes, although you can use this approach to plan
    /// multiple tasks, it may be beneficial to do one task at a time especially if you need to control the gripper in
    /// between the tasks. To do so, simply call run with that planner_config and task_description,
    /// as node->run(planner_config, task_description). There is no need to create std::vectors of them

    ///------------------------------------------------------------------///
    std::vector<cc_affordance_planner::TaskDescription> task_descriptions;
    std::vector<cc_affordance_planner::PlannerConfig> planner_configs;

    // const std::vector<DemoMotion> demo_motions = {
    //     DemoMotion::ROLL_FORWARD, DemoMotion::ROLL_BACKWARD, DemoMotion::PITCH_FORWARD, DemoMotion::PITCH_BACKWARD,
    //     DemoMotion::YAW_FORWARD,  DemoMotion::YAW_BACKWARD,  DemoMotion::APPROACH,      DemoMotion::TRANSLATION,
    //     DemoMotion::ROTATION,     DemoMotion::SCREW,         DemoMotion::CARTESIAN_GOAL};
    const std::vector<DemoMotion> demo_motions = {DemoMotion::APPROACH};

    // Populate the planner config and task descriptions for these demo motions. See the get_demo_description function
    // above for specific task descriptions.
    for (const auto &demo_motion : demo_motions)
    {

        auto [planner_config, task_description] = get_demo_description(demo_motion);
        planner_configs.push_back(planner_config);
        task_descriptions.push_back(task_description);
    }
    ///------------------------------------------------------------------///

    // For demo purposes, we'll specify a start configuration. During common practical usage, planning is done from the
    // current state of the robot, in which case there is no need to specify the start config. Simply call run without
    // it, i.e. as node->run(planner_config, task_description)
    Eigen::VectorXd HOME_CONFIG =
        (Eigen::VectorXd(7) << -3.05874e-06, 0.260055, 3.14312, -2.26992, 1.74023e-06, 0.959945, 1.57006).finished();
    cca_ros::KinematicState start_config;
    start_config.robot = HOME_CONFIG;

    // if (node->run(planner_configs, task_descriptions, start_config)) ///<-- This is where the planner is called.
    if (node->run(planner_configs, task_descriptions)) ///<-- This is where the planner is called.
    {
        RCLCPP_INFO(node->get_logger(), "Successfully called CCA action");
        node->block_until_trajectory_execution(); // Optionally, block until execution
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "CCA action failed");
        rclcpp::shutdown();
    }

    if (spinner_thread.joinable())
    {
        spinner_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}
