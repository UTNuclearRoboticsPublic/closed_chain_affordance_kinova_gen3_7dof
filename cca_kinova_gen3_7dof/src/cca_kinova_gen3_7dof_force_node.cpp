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
#include "affordance_util_ros/affordance_util_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CcaKinova : protected cca_ros::CcaRos
{
  public:
    // Type aliases
    using TwistStamped = geometry_msgs::msg::TwistStamped;

    explicit CcaKinova(const std::string &node_name, const rclcpp::NodeOptions &node_options, bool visualize_trajectory,
                       bool execute_trajectory)
        : cca_ros::CcaRos(node_name, node_options, visualize_trajectory, execute_trajectory),
          force_correction_topic_("/kinova/force_correction")
    {

        // Initialize subscriber
        force_correction_sub_ = this->create_subscription<TwistStamped>(
            force_correction_topic_, 1000,
            std::bind(&CcaKinova::force_correction_sub_cb_, this, std::placeholders::_1));
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

    void add_force_correction_to_task(
        cc_affordance_planner::TaskDescription &task_description,
        const Eigen::VectorXd current_joint_config =
            Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN()))
    {
        if (!current_joint_config.hasNaN())
        {
            // Verify force correction callback was read
            if ((body_frame_name_.empty()) || (force_correction_.hasNaN()))
            {
                throw std::runtime_error("Failed to lookup force correction data");
            }

            task_description.force_correction =
                this->transform_velocity_to_ref_frame_(force_correction_, body_frame_name_);
        }
        else
        {
            task_description.force_correction =
                this->transform_velocity_to_ref_frame_(force_correction_, current_joint_config);
        }
    }

  private:
    std::shared_ptr<cca_ros::Status> motion_status_;
    bool includes_gripper_goal_ = false;
    rclcpp::Subscription<TwistStamped>::SharedPtr force_correction_sub_; ///< Subscriber for force correction
    std::string force_correction_topic_;
    Eigen::Matrix<double, 6, 1> force_correction_ = Eigen::Matrix<double, 6, 1>::Constant(
        std::numeric_limits<double>::quiet_NaN()); // twist form of the force that needs to be corrected
    std::string body_frame_name_;

    void force_correction_sub_cb_(const TwistStamped::SharedPtr msg)
    {
        body_frame_name_ = msg->header.frame_id;

        force_correction_[0] = msg->twist.angular.x;
        force_correction_[1] = msg->twist.angular.y;
        force_correction_[2] = msg->twist.angular.z;
        force_correction_[3] = msg->twist.linear.x;
        force_correction_[4] = msg->twist.linear.y;
        force_correction_[5] = msg->twist.linear.z;
    }

    Eigen::Matrix<double, 6, 1> transform_velocity_to_ref_frame_(const Eigen::Matrix<double, 6, 1> body_twist,
                                                                 const std::string &body_frame_name)
    {
        const Eigen::Matrix4d fk = affordance_util_ros::get_htm(ref_frame_, body_frame_name, *tf_buffer_).matrix();

        return affordance_util::Adjoint(fk) * body_twist;
    }

    Eigen::Matrix<double, 6, 1> transform_velocity_to_ref_frame_(const Eigen::Matrix<double, 6, 1> body_twist,
                                                                 const Eigen::VectorXd &current_joint_config)
    {
        const Eigen::Matrix4d fk = affordance_util::FKinSpace(M_, robot_slist_, current_joint_config);
        return affordance_util::Adjoint(fk) * body_twist;
    }
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
    task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);

    // Affordance info
    task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
    task_description.affordance_info.location = Eigen::Vector3d::Zero();
    task_description.trajectory_density = 2;

    // Goals
    task_description.goal.affordance = 0.1; // Set desired goal for the affordance
    task_description.goal.grasp_pose = Eigen::Matrix4d::Identity();
    task_description.goal.grasp_pose.block<3, 1>(0, 3) = Eigen::Vector3d(-0.3, -0.3, 0.5);

    const Eigen::VectorXd HOME_CONFIG =
        (Eigen::VectorXd(7) << -3.05874e-06, 0.260055, 3.14312, -2.26992, 1.74023e-06, 0.959945, 1.57006).finished();
    try
    {
        node->add_force_correction_to_task(task_description, HOME_CONFIG);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception while trying to add force correction: %s", e.what());
    }

    cc_affordance_planner::PlannerConfig planner_config;
    cca_ros::KinematicState start_config;
    start_config.robot = HOME_CONFIG;
    ///------------------------------------------------------------------///

    // Run CCA planner and executor
    if (node->run(planner_config, task_description, start_config))
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
