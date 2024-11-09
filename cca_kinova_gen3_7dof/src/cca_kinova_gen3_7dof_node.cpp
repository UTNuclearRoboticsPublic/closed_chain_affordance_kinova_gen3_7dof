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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcaKinova>(
        "cca_ros", node_options, true,
        false); ///<-- Flip this false keyword to true to execute the trajectory on the robot
    RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

    // Spin the node so joint states can be read
    std::thread spinner_thread([node]() { rclcpp::spin(node); });

    /// REQUIRED INPUT: Task description. For quick start, the following block provides an example task description to
    /// do a simple linear motion along the z-axis from the current robot configuration. Edit as needed. See this
    /// package's demo folder or repo README.md for various other examples that cover motions including rotation, screw,
    /// cartesian goal, ee orientation jog, etc. It is also possible to plan multiple of these tasks together as a long
    /// joint trajectory.
    ///------------------------------------------------------------------///
    cc_affordance_planner::TaskDescription task_description;

    // Affordance info
    task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
    task_description.affordance_info.location = Eigen::Vector3d::Zero();

    // Goals
    task_description.goal.affordance = 0.5; // Set desired goal for the affordance

    cc_affordance_planner::PlannerConfig planner_config;
    ///------------------------------------------------------------------///

    // Run CCA planner and executor
    if (node->run(planner_config, task_description))
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
