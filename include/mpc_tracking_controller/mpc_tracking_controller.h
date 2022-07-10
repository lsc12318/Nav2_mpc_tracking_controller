//
// Created by nubot on 2022/7/4.
//

#ifndef MPC_TRACKING_CONTROLLER_MPC_TRACKING_CONTROLLER_H
#define MPC_TRACKING_CONTROLLER_MPC_TRACKING_CONTROLLER_H
#include "mpc_plannner.h"

#include <string.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <stdio.h>

#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std;

namespace mpc_tracking_controller
{
    class MpcTrackingController: public nav2_core::Controller
    {
    public:
        MpcTrackingController() = default;
        ~MpcTrackingController() override = default;

        /**
        * @brief Configure controller state machine
        * @param parent WeakPtr to node
        * @param name Name of plugin
        * @param tf TF buffer
        * @param costmap_ros Costmap2DROS object of environment
        */
        void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;

        void reconfig_mpc();

        /**
         * @brief Cleanup controller state machine
         */
        void cleanup() override;

        /**
         * @brief Activate controller state machine
         */
        void activate() override;

        /**
         * @brief Deactivate controller state machine
         */
        void deactivate() override;

        /**
      * @brief Compute the best command given the current pose and velocity, with possible debug information
      *
      * Same as above computeVelocityCommands, but with debug results.
      * If the results pointer is not null, additional information about the twists
      * evaluated will be in results after the call.
      *
      * @param pose      Current robot pose
      * @param velocity  Current robot velocity
      * @param goal_checker   Ptr to the goal checker for this task in case useful in computing commands
      * @return          Best command
      */
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
                const geometry_msgs::msg::PoseStamped & pose,
                const geometry_msgs::msg::Twist & velocity,
                nav2_core::GoalChecker * /*goal_checker*/) override;


        /**
      * @brief Actually implement of MPC
      *
      *
      * @param pose      Current robot pose
      * @param velocity  Current robot velocity
      * @param transformed_plan   Global plan at the frame as same as robot
      * @return          Best command
      */
        geometry_msgs::msg::TwistStamped do_mpc(const geometry_msgs::msg::PoseStamped & pose,
                              const geometry_msgs::msg::Twist & velocity,
                              nav_msgs::msg::Path & transformed_plan );

        /**
       * @brief nav2_core setPlan - Sets the global plan
       * @param path The global plan
       */
        void setPlan(const nav_msgs::msg::Path & path) override;

        /**
       * @brief Limits the maximum linear speed of the robot.
       * @param speed_limit expressed in absolute value (in m/s)
       * or in percentage from maximum robot speed.
       * @param percentage Setting speed limit in percentage if true
       * or in absolute values in false case.
       */
        void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

        /**
       * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
       * @param pose pose to transform
       * @return Path in new frame
       */
        nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

        /**
        * @brief Transform a pose to another frame.
        * @param frame Frame ID to transform to
        * @param in_pose Pose input to transform
        * @param out_pose transformed output
        * @return bool if successful
        */
        bool transformPose(
                const std::string frame,
                const geometry_msgs::msg::PoseStamped & in_pose,
                geometry_msgs::msg::PoseStamped & out_pose) const;

    private:
        //for ros
        nav_msgs::msg::Path global_plan_;

        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_path_pub_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D * costmap_;
        rclcpp::Logger logger_ {rclcpp::get_logger("MPCController")};
        rclcpp::Clock::SharedPtr clock_;
        tf2::Duration transform_tolerance_;

        double transform_tolerance;
        double control_frequency;
        double control_duration;
        double goal_dist_tol_;

        //for mpc
        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuatotions.
        //vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

        MPC _mpc;
        vector<double> mpc_x;
        vector<double> mpc_y;
        vector<double> mpc_theta;


        // Parameters for mpc slover
        std::map<string, double> params;

        bool pid_mode{false};
        bool mpc_mode{true};
        int _mpc_steps;
        double _max_angvel;
        double _max_throttle;
        double _bound_value;

        // Parameters for FG_eval
        double  _dt,_ref_cte, _ref_etheta, _ref_vel, _w_cte, _w_etheta, _w_vel,
                _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;

        //Parameters for downsample path and control command

        double  _w{0.0}, _throttle{0.0}, _speed{0.0}, _max_speed{0.8};

        double _pathLength{1.5}, _goalRadius{0.5}, _waypointsDist{-1.0};
        int _downSampling{0};
        bool _debug_info, _delay_mode;
        double polyeval(Eigen::VectorXd coeffs, double x);
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
    };



}









#endif //MPC_TRACKING_CONTROLLER_MPC_TRACKING_CONTROLLER_H
