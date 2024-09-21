/***************************/
// Author: Crasun Jans
// Description: The following enables the manipulator-integrated Spot to autonomously undock, walk to a chair, grab it,
// move it out of the way, and go back and dock.
/***************************/
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <algorithm>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cca_ros/cca_ros.hpp>
#include <optional>
#include <spot_msgs/action/walk_to.hpp>
#include <spot_msgs/srv/dock.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;
class WalkToAndMoveChair : public cca_ros::CcaRos
{
  public:
    using WalkTo = spot_msgs::action::WalkTo;
    using GoalHandleWalkTo = rclcpp_action::ClientGoalHandle<WalkTo>;

    enum DemoMotion
    {
        PREAPPROACH_FORWARD,
        APPROACH,
        GRASPTUNE_FORWARD,
        GRASPTUNE_UPWARD,
        AFFORDANCE,
        PUSH,
        RETRACT
    };

    explicit WalkToAndMoveChair(const std::string &node_name, const rclcpp::NodeOptions &node_options,
                                bool visualize_trajectory, bool execute_trajectory)
        : cca_ros::CcaRos(node_name, node_options, visualize_trajectory, execute_trajectory),
          walk_action_server_name_("/spot_driver/walk_to"),
          gripper_open_server_name_("/spot_manipulation_driver/open_gripper"),
          gripper_close_server_name_("/spot_manipulation_driver/close_gripper"),
          mini_unstow_server_name_("/spot_manipulation_driver/mini_unstow_arm"),
          stow_server_name_("/spot_manipulation_driver/stow_arm"),
          undock_server_name_("/spot_driver/undock"),
          dock_server_name_("/spot_driver/dock")
    {
        // Initialize clients
        gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_open_server_name_);
        gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>(gripper_close_server_name_);
        mini_unstow_client_ = this->create_client<std_srvs::srv::Trigger>(mini_unstow_server_name_);
        stow_client_ = this->create_client<std_srvs::srv::Trigger>(stow_server_name_);
        undock_client_ = this->create_client<std_srvs::srv::Trigger>(undock_server_name_);
        dock_client_ = this->create_client<spot_msgs::srv::Dock>(dock_server_name_);
        walk_action_client_ = rclcpp_action::create_client<spot_msgs::action::WalkTo>(this, walk_action_server_name_);

        // Construct buffer to lookup chair location from apriltag using tf data
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    ~WalkToAndMoveChair() { rclcpp::shutdown(); }
    void run_demo()
    {
        std::optional<geometry_msgs::msg::PoseStamped> chair_lookup_result = lookup_chair_walk_goal_();
        if (!chair_lookup_result.has_value())
        {

            RCLCPP_ERROR(this->get_logger(), "Chair lookup failed");
            return;
        }
        geometry_msgs::msg::PoseStamped chair_walk_goal_pose = chair_lookup_result.value();

        /********************************************************/
        RCLCPP_INFO(this->get_logger(), "Undocking robot");
        if (!undock_robot_())
        {

            RCLCPP_ERROR(this->get_logger(), "Undock failed");
            return;
        }
        /********************************************************/

        RCLCPP_INFO(this->get_logger(), "Capturing robot start pose");
        // Capture robot's current pose to walk back to it later
        const Eigen::Isometry3d htm_start_pose =
            affordance_util_ros::get_htm(fixed_frame_, robot_navigation_frame_, *tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Walking to chair");
        if (!walk_to_chair_(chair_walk_goal_pose))
        {
            RCLCPP_ERROR(this->get_logger(), "Walking to chair failed");
            /* return; */
        }
        chair_lookup_result = lookup_chair_walk_goal_();
        if (!chair_lookup_result.has_value())
        {

            RCLCPP_ERROR(this->get_logger(), "Chair lookup failed");
            return;
        }
        chair_walk_goal_pose = chair_lookup_result.value();

        walk_result_available_ = false;
        walk_success_ = false;
        RCLCPP_INFO(this->get_logger(), "Walking to chair again");
        if (!walk_to_chair_(chair_walk_goal_pose))
        {
            RCLCPP_ERROR(this->get_logger(), "Walking to chair again failed");
            return;
        }
        /********************************************************/

        RCLCPP_INFO(this->get_logger(), "Mini-unstowing arm");
        if (!mini_unstow_arm_())
        {

            RCLCPP_ERROR(this->get_logger(), "Mini unstow failed");
            return;
        }

        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::PREAPPROACH_FORWARD, "preapproach-forward motion"))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/
        /* const Eigen::Matrix4d approach_pose = get_approach_pose_(); */
        /* if (!(this->execute_motion_(DemoMotion::APPROACH, "approach motion", approach_pose))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Opening gripper"); */
        /* if (!open_gripper_()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Opening gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::GRASPTUNE_FORWARD, "grasp-tune-forward motion"))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::GRASPTUNE_UPWARD, "grasp-tune-upward motion"))) */
        /* { */
        /*     return; */
        /* } */
        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Closing gripper"); */
        /* if (!close_gripper_()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Closing gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::AFFORDANCE, "affordance motion"))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::PUSH, "push motion"))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/

        /* RCLCPP_INFO(this->get_logger(), "Opening gripper"); */
        /* if (!open_gripper_()) */
        /* { */

        /*     RCLCPP_ERROR(this->get_logger(), "Opening gripper failed"); */
        /*     return; */
        /* } */
        /********************************************************/
        /* if (!(this->execute_motion_(DemoMotion::RETRACT, "retract motion"))) */
        /* { */
        /*     return; */
        /* } */

        /********************************************************/
        // If multiple arm motions at once, all in this case:
        const Eigen::Matrix4d approach_pose = get_approach_pose_();
        const std::vector<DemoMotion> demo_motions = {DemoMotion::PREAPPROACH_FORWARD,
                                                      DemoMotion::APPROACH,
                                                      DemoMotion::GRASPTUNE_FORWARD,
                                                      DemoMotion::GRASPTUNE_UPWARD,
                                                      DemoMotion::AFFORDANCE,
                                                      DemoMotion::PUSH,
                                                      DemoMotion::RETRACT};

        if (!(this->execute_motion_(demo_motions, "multiple motions", approach_pose)))
        {
            return;
        }
        /********************************************************/

        RCLCPP_INFO(this->get_logger(), "Stowing arm");
        if (!stow_arm_())
        {

            RCLCPP_ERROR(this->get_logger(), "Stow failed");
            return;
        }
        /********************************************************/

        if (!close_gripper_())
        {

            RCLCPP_ERROR(this->get_logger(), "Closing gripper failed");
            return;
        }
        /********************************************************/

        walk_result_available_ = false;
        walk_success_ = false;
        RCLCPP_INFO(this->get_logger(), "Walking back to start pose");
        if (!walk_back_to_start_pose_(htm_start_pose.matrix()))
        {
            RCLCPP_ERROR(this->get_logger(), "Walking back to start pose failed");
            return;
        }
        /********************************************************/

        RCLCPP_INFO(this->get_logger(), "Docking robot");
        if (!dock_robot_())
        {

            RCLCPP_ERROR(this->get_logger(), "Dock failed");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully concluded demo");

        rclcpp::shutdown();
    }

  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // buffer to lookup tf data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // Clients
    rclcpp_action::Client<spot_msgs::action::WalkTo>::SharedPtr walk_action_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_open_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_close_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mini_unstow_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stow_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undock_client_;
    rclcpp::Client<spot_msgs::srv::Dock>::SharedPtr dock_client_;
    // Client names
    std::string walk_action_server_name_;
    std::string gripper_open_server_name_;
    std::string gripper_close_server_name_;
    std::string mini_unstow_server_name_;
    std::string stow_server_name_;
    std::string undock_server_name_;
    std::string dock_server_name_;

    const std::string ref_frame_ = "arm0_base_link";
    const std::string tool_frame_ = "arm0_tool0";
    const std::string chair_frame_ = "affordance_frame"; // Name of the AprilTag frame to locate the chair
    const std::string fixed_frame_ = "odom";
    const std::string robot_navigation_frame_ = "base_footprint";

    const Eigen::Matrix4d htm_c2wg_ =
        (Eigen::Matrix4d() << 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.4, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to walk goal

    const Eigen::Matrix4d htm_c2a_ =
        (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.36, 0.0, 0.0, 1.0, 0.50, 0.0, 0.0, 0.0, 1.0)
            .finished(); // chair to approach
    const double approach_pose_z_offset_ = -0.3;

    bool walk_result_available_ = false;
    bool walk_success_ = false;
    std_srvs::srv::Trigger::Request::SharedPtr trigger_req_ = std::make_shared<std_srvs::srv::Trigger::Request>();

    const Eigen::VectorXd robot_start_config_ =
        /* (Eigen::VectorXd(6) << -1.49419, -1.09419, 2.2496, -0.567882, -0.796551, 0.396139).finished(); */
        (Eigen::VectorXd(6) << 0.0, -1.09419, 2.2496, -0.567882, -0.796551, 0.396139).finished(); // For testing
    const double gripper_start_config_ = 0.0;

    // Methods
    bool execute_motion_(const DemoMotion demo_motion, const std::string &motion_name,
                         const Eigen::Matrix4d &approach_pose = Eigen::Matrix4d())
    {
        // Start motion status as unknown
        auto motion_status = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);

        RCLCPP_INFO(this->get_logger(), "Executing %s motion", motion_name.c_str());

        auto [planner_config, task_description] =
            this->get_planner_config_and_task_description_(demo_motion, approach_pose);

        // Execute the specified motion
        /* cca_ros::KinematicState start_config; */
        /* start_config.robot = robot_start_config_; */
        /* start_config.gripper = gripper_start_config_; */
        /* if (!(this->run_cc_affordance_planner(planner_config, task_description, motion_status, */
        /* start_config))) // Call with robot_start_config for testing */
        if (!(this->run_cc_affordance_planner(planner_config, task_description, motion_status)))
        {
            RCLCPP_ERROR(this->get_logger(), "%s motion failed", motion_name.c_str());
            return false;
        }

        // Check status
        rclcpp::Rate loop_rate(4);
        while (*motion_status != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status == cca_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "%s motion was interrupted mid-execution.", motion_name.c_str());
                return false;
            }

            loop_rate.sleep();
        }
        if (!std::isnan(task_description.goal.gripper))
        {
            // join status monitoring thread if task includes gripper goal
            this->cleanup_between_calls();
        }
        return true;
    }

    bool execute_motion_(const std::vector<DemoMotion> &demo_motions, const std::string &motion_name,
                         const Eigen::Matrix4d &approach_pose = Eigen::Matrix4d())
    {
        // Start motion status as unknown
        auto motion_status = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);

        RCLCPP_INFO(this->get_logger(), "Executing %s motion", motion_name.c_str());

        // Compose planner configs and task descriptions for various motion tasks
        std::vector<cc_affordance_planner::PlannerConfig> planner_configs;
        std::vector<cc_affordance_planner::TaskDescription> task_descriptions;
        for (const auto &demo_motion : demo_motions)
        {

            auto [planner_config, task_description] =
                this->get_planner_config_and_task_description_(demo_motion, approach_pose);
            planner_configs.push_back(planner_config);
            task_descriptions.push_back(task_description);
        }

        // Execute the specified motion
        /* cca_ros::KinematicState start_config; */
        /* start_config.robot = robot_start_config_; */
        /* start_config.gripper = gripper_start_config_; */
        /* if (!(this->run_cc_affordance_planner(planner_configs, task_descriptions, motion_status, start_config,
         */
        /*                                       gripper_start_config_))) */
        if (!(this->run_cc_affordance_planner(planner_configs, task_descriptions, motion_status)))
        {
            RCLCPP_ERROR(this->get_logger(), "%s motion failed", motion_name.c_str());
            return false;
        }

        // Check status
        rclcpp::Rate loop_rate(4);
        while (*motion_status != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status == cca_ros::Status::UNKNOWN)
            {

                RCLCPP_ERROR(this->get_logger(), "%s motion was interrupted mid-execution.", motion_name.c_str());
                return false;
            }

            loop_rate.sleep();
        }
        return true;
    }

    bool undock_robot_()
    {

        // Wait for service to be available
        while (!undock_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             undock_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", undock_server_name_.c_str());
        }

        auto result = undock_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", undock_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", undock_server_name_.c_str());
            return false;
        }
    }

    std::optional<geometry_msgs::msg::PoseStamped> lookup_chair_walk_goal_()
    {

        //  Lookup and compute walk goal
        rclcpp::Rate tag_read_rate(4);
        std::vector<Eigen::Matrix4d> htm_r2c_vec;
        for (int i = 0; i < 10; i++)
        {
            const Eigen::Isometry3d htm_r2c =
                affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair
            if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
                return std::nullopt;
            }
            affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair
            htm_r2c_vec.push_back(htm_r2c.matrix());
            tag_read_rate.sleep();
        }

        // Compute the median matrix to combat reading fluctuations
        Eigen::Matrix4d htm_r2c_matrix = findMedianNormMatrix_(htm_r2c_vec);
        if (htm_r2c_matrix.isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "The median matrix is close to identity");
            return std::nullopt;
        }

        const Eigen::Matrix4d htm_wr2wg = htm_r2c_matrix * htm_c2wg_;
        const Eigen::Quaterniond quat_wr2wg(htm_wr2wg.block<3, 3>(0, 0)); // quaternion representation

        // Fill out walk goal message
        geometry_msgs::msg::PoseStamped walk_goal_pose;
        walk_goal_pose.header.frame_id = ref_frame_;
        walk_goal_pose.header.stamp = this->get_clock()->now();

        walk_goal_pose.pose.position.x = htm_wr2wg(0, 3);
        walk_goal_pose.pose.position.y = htm_wr2wg(1, 3);
        walk_goal_pose.pose.orientation.x = quat_wr2wg.x();
        walk_goal_pose.pose.orientation.y = quat_wr2wg.y();
        walk_goal_pose.pose.orientation.z = quat_wr2wg.z();
        walk_goal_pose.pose.orientation.w = quat_wr2wg.w();

        return walk_goal_pose;
    }
    bool walk_to_chair_(const geometry_msgs::msg::PoseStamped &walk_goal_pose)
    {

        /* //  Lookup and compute walk goal */
        /* rclcpp::Rate tag_read_rate(4); */
        /* std::vector<Eigen::Matrix4d> htm_r2c_vec; */
        /* for (int i = 0; i < 10; i++) */
        /* { */
        /*     const Eigen::Isometry3d htm_r2c = */
        /*         affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair */
        /*     if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity())) */
        /*     { */
        /*         RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
         */
        /*         return false; */
        /*     } */
        /*     affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // ref frame to chair */
        /*     htm_r2c_vec.push_back(htm_r2c.matrix()); */
        /*     tag_read_rate.sleep(); */
        /* } */

        /* // Compute the median matrix to combat reading fluctuations */
        /* Eigen::Matrix4d htm_r2c_matrix = findMedianNormMatrix_(htm_r2c_vec); */
        /* if (htm_r2c_matrix.isApprox(Eigen::Matrix4d::Identity())) */
        /* { */
        /*     RCLCPP_ERROR(this->get_logger(), "The median matrix is close to identity"); */
        /*     return false; */
        /* } */

        /* const Eigen::Matrix4d htm_wr2wg = htm_r2c_matrix * htm_c2wg_; */
        /* const Eigen::Matrix4d htm_wr2wg = htm_r2c.matrix() * htm_c2wg_; */

        /* const Eigen::Quaterniond quat_wr2wg(htm_wr2wg.block<3, 3>(0, 0)); // quaternion representation */

        /* // Fill out walk goal message */
        /* geometry_msgs::msg::PoseStamped walk_goal_pose; */
        /* walk_goal_pose.header.frame_id = ref_frame_; */
        /* walk_goal_pose.header.stamp = this->get_clock()->now(); */

        /* walk_goal_pose.pose.position.x = htm_wr2wg(0, 3); */
        /* walk_goal_pose.pose.position.y = htm_wr2wg(1, 3); */
        /* walk_goal_pose.pose.orientation.x = quat_wr2wg.x(); */
        /* walk_goal_pose.pose.orientation.y = quat_wr2wg.y(); */
        /* walk_goal_pose.pose.orientation.z = quat_wr2wg.z(); */
        /* walk_goal_pose.pose.orientation.w = quat_wr2wg.w(); */

        WalkTo::Goal walk_goal;
        walk_goal.target_pose = walk_goal_pose;
        walk_goal.maximum_movement_time = 10.0;

        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto send_goal_options = rclcpp_action::Client<WalkTo>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WalkToAndMoveChair::walk_goal_response_callback_, this, _1);
        send_goal_options.result_callback = std::bind(&WalkToAndMoveChair::walk_result_callback_, this, _1);
        this->walk_action_client_->async_send_goal(walk_goal, send_goal_options);
        rclcpp::Rate loop_rate(4);
        while (!walk_result_available_)
        {
            loop_rate.sleep();
        }
        return walk_success_;
    }
    void walk_goal_response_callback_(const GoalHandleWalkTo::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by %s action server", walk_action_server_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by %s action server, waiting for result",
                        walk_action_server_name_.c_str());
        }
    }

    void walk_result_callback_(const GoalHandleWalkTo::WrappedResult &result)
    {
        walk_result_available_ = true;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "%s action server goal was aborted", walk_action_server_name_.c_str());
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "%s action server goal was canceled", walk_action_server_name_.c_str());
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "%s action server unknown result code", walk_action_server_name_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "%s action server call concluded", walk_action_server_name_.c_str());
        walk_success_ = true;
    }

    bool mini_unstow_arm_()
    {

        // Wait for service to be available
        while (!mini_unstow_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             mini_unstow_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        mini_unstow_server_name_.c_str());
        }

        auto result = mini_unstow_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", mini_unstow_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", mini_unstow_server_name_.c_str());
            return false;
        }
    }

    bool stow_arm_()
    {

        // Wait for service to be available
        while (!stow_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             stow_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", stow_server_name_.c_str());
        }

        auto result = stow_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", stow_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", stow_server_name_.c_str());
            return false;
        }
    }
    bool open_gripper_()
    {

        // Wait for service to be available
        while (!gripper_open_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             gripper_open_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        gripper_open_server_name_.c_str());
        }

        auto result = gripper_open_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", gripper_open_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", gripper_open_server_name_.c_str());
            return false;
        }
    }
    bool close_gripper_()
    {

        // Wait for service to be available
        while (!gripper_close_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             gripper_close_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...",
                        gripper_close_server_name_.c_str());
        }

        auto result = gripper_close_client_->async_send_request(trigger_req_);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", gripper_close_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", gripper_close_server_name_.c_str());
            return false;
        }
    }

    void publish_transform_(const Eigen::Matrix4d &htm)
    {
        geometry_msgs::msg::TransformStamped t;

        Eigen::Quaterniond htm_quat(htm.block<3, 3>(0, 0));

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = ref_frame_;
        t.child_frame_id = "approach_frame";
        t.transform.translation.x = htm(0, 3);
        t.transform.translation.y = htm(1, 3);
        t.transform.translation.z = htm(2, 3);
        t.transform.rotation.x = htm_quat.x();
        t.transform.rotation.y = htm_quat.y();
        t.transform.rotation.z = htm_quat.z();
        t.transform.rotation.w = htm_quat.w();

        rclcpp::Rate loop_rate(4);
        for (int i = 0; i <= 10; i++)
        {
            tf_broadcaster_->sendTransform(t);
            loop_rate.sleep();
        }
    }

    bool walk_back_to_start_pose_(const Eigen::Matrix4d &htm_start_pose)
    {

        //  Lookup and compute walk goal
        const Eigen::Quaterniond quat_start_pose(htm_start_pose.block<3, 3>(0, 0)); // quaternion representation

        // Fill out walk goal message
        geometry_msgs::msg::PoseStamped walk_goal_pose;
        walk_goal_pose.header.frame_id = fixed_frame_;
        walk_goal_pose.header.stamp = this->get_clock()->now();

        walk_goal_pose.pose.position.x = htm_start_pose(0, 3);
        walk_goal_pose.pose.position.y = htm_start_pose(1, 3);
        walk_goal_pose.pose.orientation.x = quat_start_pose.x();
        walk_goal_pose.pose.orientation.y = quat_start_pose.y();
        walk_goal_pose.pose.orientation.z = quat_start_pose.z();
        walk_goal_pose.pose.orientation.w = quat_start_pose.w();

        WalkTo::Goal walk_goal;
        walk_goal.target_pose = walk_goal_pose;
        walk_goal.maximum_movement_time = 10.0;

        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto send_goal_options = rclcpp_action::Client<WalkTo>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&WalkToAndMoveChair::walk_goal_response_callback_, this, _1);
        send_goal_options.result_callback = std::bind(&WalkToAndMoveChair::walk_result_callback_, this, _1);
        this->walk_action_client_->async_send_goal(walk_goal, send_goal_options);
        rclcpp::Rate loop_rate(4);
        while (!walk_result_available_)
        {
            loop_rate.sleep();
        }
        return walk_success_;
    }

    bool dock_robot_()
    {

        // Wait for service to be available
        while (!dock_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for %s service. Exiting.",
                             dock_server_name_.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), " %s service not available, waiting again...", dock_server_name_.c_str());
        }

        auto dock_req = std::make_shared<spot_msgs::srv::Dock::Request>();
        dock_req->dock_id = 520;
        auto result = dock_client_->async_send_request(dock_req);
        auto response = result.get();
        // Read response
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "%s service called successfully", dock_server_name_.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call %s service", dock_server_name_.c_str());
            return false;
        }
    }

    // Function to find the matrix closest to the median norm
    Eigen::Matrix4d findMedianNormMatrix_(const std::vector<Eigen::Matrix4d> &matrices)
    {

        std::vector<double> norms;
        norms.reserve(matrices.size());

        for (const auto &matrix : matrices)
        {
            norms.push_back(matrix.norm());
        }

        // Sort norms and find the median
        std::sort(norms.begin(), norms.end());
        double medianNorm;
        size_t numMatrices = norms.size();
        if (numMatrices % 2 == 0)
        {
            medianNorm = (norms[numMatrices / 2 - 1] + norms[numMatrices / 2]) / 2.0;
        }
        else
        {
            medianNorm = norms[numMatrices / 2];
        }

        // Find the matrix whose norm is closest to the median norm
        double minDiff = std::numeric_limits<double>::max();
        Eigen::Matrix4d closestMatrix;

        for (const auto &matrix : matrices)
        {
            double norm = matrix.norm();
            double diff = std::abs(norm - medianNorm);

            if (diff < minDiff)
            {
                minDiff = diff;
                closestMatrix = matrix;
            }
        }

        return closestMatrix;
    }

    std::pair<cc_affordance_planner::PlannerConfig, cc_affordance_planner::TaskDescription>
    get_planner_config_and_task_description_(const DemoMotion &demo_motion,
                                             const Eigen::Matrix4d &approach_pose = Eigen::Matrix4d())
    {
        // Default planner info
        cc_affordance_planner::PlannerConfig planner_config;

        affordance_util::ScrewInfo aff;
        Eigen::VectorXd aff_goal;
        cc_affordance_planner::TaskDescription task_description;
        task_description.trajectory_density = 5;

        switch (demo_motion)
        {
        case DemoMotion::PREAPPROACH_FORWARD:
            // Affordance info
            aff.type = affordance_util::ScrewType::TRANSLATION;
            aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

            // Task description
            task_description.affordance_info = aff;
            task_description.goal.affordance = 0.24;
            task_description.goal.gripper = 0.0;
            break;

        case DemoMotion::APPROACH:
            // Planner info
            planner_config.accuracy = 5.0 / 100.0;

            // Affordance info
            aff.type = affordance_util::ScrewType::ROTATION;
            aff.axis = Eigen::Vector3d(0, 0, 1);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
            aff_goal = (Eigen::VectorXd(1) << 0).finished();

            // Task description
            task_description.motion_type = cc_affordance_planner::MotionType::APPROACH;
            task_description.gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
            task_description.affordance_info = aff;
            task_description.trajectory_density = 10;
            task_description.goal.affordance = 1e-7;
            task_description.goal.ee_orientation = Eigen::Vector3d::Constant(1e-7);
            task_description.goal.grasp_pose = approach_pose;
            task_description.goal.gripper = -(1.0 / 2.0) * M_PI;
            break;

        case DemoMotion::GRASPTUNE_FORWARD:
            // Affordance info
            aff.type = affordance_util::ScrewType::TRANSLATION;
            aff.axis = Eigen::Vector3d(1.0, 0.0, 0.0);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

            // Task description
            task_description.affordance_info = aff;
            task_description.trajectory_density = 10;
            task_description.goal.affordance = 0.25;
            task_description.goal.gripper = -(1.0 / 2.0) * M_PI;
            break;

        case DemoMotion::GRASPTUNE_UPWARD:
            // Affordance info
            aff.type = affordance_util::ScrewType::TRANSLATION;
            aff.axis = Eigen::Vector3d(0.0, 0.0, 1.0);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

            // Task description
            task_description.vir_screw_order = affordance_util::VirtualScrewOrder::NONE;
            task_description.gripper_goal_type = affordance_util::GripperGoalType::CONTINUOUS;
            task_description.affordance_info = aff;
            task_description.goal.affordance = 0.08;
            task_description.goal.gripper = 0;
            break;

        case DemoMotion::AFFORDANCE:

            // Affordance info
            aff.type = affordance_util::ScrewType::ROTATION;
            aff.axis = Eigen::Vector3d(0, 0, -1);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

            // Task description
            task_description.trajectory_density = 10;
            task_description.affordance_info = aff;
            task_description.goal.affordance = (1.0 / 2.0) * M_PI;
            task_description.goal.gripper = 0.0;
            break;

        case DemoMotion::PUSH:
            // Affordance info
            aff.type = affordance_util::ScrewType::TRANSLATION;
            aff.axis = Eigen::Vector3d(0.0, 1.0, 0.0);
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);

            // Task description
            task_description.affordance_info = aff;
            task_description.goal.affordance = 0.08;
            task_description.goal.gripper = 0.0;
            break;

        case DemoMotion::RETRACT:
            // Affordance info
            aff.type = affordance_util::ScrewType::TRANSLATION;
            aff.location = Eigen::Vector3d(0.0, 0.0, 0.0);
            aff.axis = Eigen::Vector3d(0.0, -1.0, 0.0);

            // Task description
            task_description.affordance_info = aff;
            task_description.goal.affordance = 0.16;
            task_description.goal.gripper = -(1.0 / 2.0) * M_PI;
            break;
        }

        return std::make_pair(planner_config, task_description);
    }
    Eigen::Matrix4d get_approach_pose_()
    {

        // Lookup and compute approach pose
        const Eigen::Isometry3d htm_r2c =
            affordance_util_ros::get_htm(ref_frame_, chair_frame_, *tf_buffer_); // reference frame to chair

        if (htm_r2c.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.", chair_frame_.c_str());
            return Eigen::Matrix4d();
        }

        Eigen::Matrix4d approach_pose = htm_r2c.matrix() * htm_c2a_; // adjust y offset in the chair frame

        approach_pose(2, 3) = htm_r2c.matrix()(2, 3) + approach_pose_z_offset_; // adjust z offset in the ref
        approach_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        /* Eigen::Matrix4d approach_pose; */
        /* approach_pose << 0.998453, 0.0378028, 0.0407843, 0.529228, -0.0380367, 0.999264, 0.00497515, -0.16148, */
        /*     -0.0405662, -0.00651876, 0.999156, 0.100135, 0, 0, 0, 1; */

        // Change the approach pose orientation about the x axis
        Eigen::Matrix4d rot_x;
        double theta = 0.0;
        /* double theta = M_PI / 2; */
        /* double theta = (2.0/3.0)*M_PI; */
        rot_x << 1, 0, 0, 0, 0, cos(theta), -sin(theta), 0, 0, sin(theta), cos(theta), 0, 0, 0, 0, 1;
        approach_pose = approach_pose * rot_x;

        return approach_pose;
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<WalkToAndMoveChair>("cca_ros", node_options, true, true);

    // Start spinning the node in a separate thread so we could do things like reading parameters and joint
    // states inside the node
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    // Run the demo
    node->run_demo();
    spinner_thread.join(); // join the spinning thread and exit
    return 0;
}
