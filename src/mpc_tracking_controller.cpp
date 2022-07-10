//
// Created by nubot on 2022/7/4.
//
#include "mpc_tracking_controller/mpc_tracking_controller.h"


#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp" //for parameters
#include "nav2_util/geometry_utils.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "Eigen/QR"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
using namespace nav2_costmap_2d;
using namespace std;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace mpc_tracking_controller{

    void MpcTrackingController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                                          const std::shared_ptr<tf2_ros::Buffer> &tf,
                                          const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) {
        //initial for ros2
        auto node = parent.lock();
        if(!node)
        {
            throw nav2_core::PlannerException("Unable to lock node!");
        }
        this->costmap_ros_ = costmap_ros;
        this->costmap_ = this->costmap_ros_->getCostmap();

        this->tf_ = tf;
        this->plugin_name_ = name;

        //get log and time
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        //declear parameters
        declare_parameter_if_not_declared(node,plugin_name_+".mpc_steps",rclcpp::ParameterValue(20));
        declare_parameter_if_not_declared(node,plugin_name_+".max_angvel",rclcpp::ParameterValue(2.0));
        declare_parameter_if_not_declared(node,plugin_name_+".max_throttle",rclcpp::ParameterValue(0.8));
        declare_parameter_if_not_declared(node,plugin_name_+".bound_value",rclcpp::ParameterValue(1.0e3));
        declare_parameter_if_not_declared(node,plugin_name_+".dt",rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(node,plugin_name_+".ref_cte",rclcpp::ParameterValue(0.01));
        declare_parameter_if_not_declared(node,plugin_name_+".ref_etheta",rclcpp::ParameterValue(0.0));
        declare_parameter_if_not_declared(node,plugin_name_+".ref_vel",rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_cte",rclcpp::ParameterValue(200.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_etheta",rclcpp::ParameterValue(50.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_vel",rclcpp::ParameterValue(100.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_angvel",rclcpp::ParameterValue(10.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_accel",rclcpp::ParameterValue(10.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_angvel_d",rclcpp::ParameterValue(100.0));
        declare_parameter_if_not_declared(node,plugin_name_+".w_accel_d",rclcpp::ParameterValue(100.0));


        declare_parameter_if_not_declared(node,plugin_name_+".pathLength",rclcpp::ParameterValue(1.5));
        // declare_parameter_if_not_declared(node,plugin_name_+".waypointsDist",rclcpp::ParameterValue(-1.0));


        declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(node, plugin_name_ + ".control_frequency", rclcpp::ParameterValue(20.0));
        declare_parameter_if_not_declared(node, plugin_name_ + ".delay_mode", rclcpp::ParameterValue(false));
        //get parameters


        node->get_parameter(plugin_name_+".mpc_steps",_mpc_steps);
        node->get_parameter(plugin_name_+".max_angvel",_max_angvel);
        node->get_parameter(plugin_name_+".max_throttle",_max_throttle);
        node->get_parameter(plugin_name_+".bound_value",_bound_value);
        node->get_parameter(plugin_name_+".dt",_dt);
        node->get_parameter(plugin_name_+".ref_cte",_ref_cte);
        node->get_parameter(plugin_name_+".ref_etheta",_ref_etheta);
        node->get_parameter(plugin_name_+".ref_vel",_ref_vel);
        node->get_parameter(plugin_name_+".w_cte",_w_cte);
        node->get_parameter(plugin_name_+".w_etheta",_w_etheta);
        node->get_parameter(plugin_name_+".w_vel",_w_vel);
        node->get_parameter(plugin_name_+".w_angvel",_w_angvel);
        node->get_parameter(plugin_name_+".w_accel",_w_accel);
        node->get_parameter(plugin_name_+".w_angvel_d",_w_angvel_d);
        node->get_parameter(plugin_name_+".w_accel_d",_w_accel_d);
        node->get_parameter(plugin_name_+".pathLength",_pathLength);
        node->get_parameter(plugin_name_+".waypointsDist",_waypointsDist);
        node->get_parameter(plugin_name_+".delay_mode",_delay_mode);



        node->get_parameter( plugin_name_ + ".transform_tolerance", transform_tolerance);
        node->get_parameter( plugin_name_ + ".control_frequency", control_frequency);



        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
        control_duration = 1.0 / control_frequency;
        goal_dist_tol_ = 0.25;
        global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
        local_path_pub_ =  node->create_publisher<nav_msgs::msg::Path>("mpc_plan",1);

        _mpc = MPC();
        reconfig_mpc();
        _mpc.LoadParams(this->params);

    }

    void MpcTrackingController::reconfig_mpc() {
        this->params["STEPS"] = _mpc_steps;
        this->params["ANGVEL"] = _max_angvel;
        this->params["MAXTHR"] = _max_throttle;
        this->params["BOUND"] = _bound_value;

        this->params["DT"] = _dt;
        this->params["REF_CTE"] = _ref_cte;
        this->params["REF_ETHETA"] = _ref_etheta;
        this->params["REF_V"] = _ref_vel;


        this->params["W_CTE"] = _w_cte;
        this->params["W_EPSI"] = _w_etheta;
        this->params["W_V"] = _w_vel;
        this->params["W_ANGVEL"] = _w_angvel;
        this->params["W_A"] = _w_accel;
        this->params["w_DANGVEL"] = _w_angvel_d;
        this->params["W_DA"] = _w_accel_d;


    }

    void MpcTrackingController::cleanup()
    {
        RCLCPP_INFO(logger_,"Cleaning up controller: %s of type"
                            " mpc_tracking_controller::MpcTrackingController",
                    plugin_name_.c_str());
        global_path_pub_.reset();
        local_path_pub_.reset();
    }

    void MpcTrackingController::activate()
    {
        RCLCPP_INFO(logger_,"Activating up controller: %s of type"
                            " mpc_tracking_controller::MpcTrackingController",
                    plugin_name_.c_str());
        global_path_pub_->on_activate();
        local_path_pub_->on_activate();

    }

    void MpcTrackingController::deactivate()
    {
        RCLCPP_INFO(logger_,"Deactivating up controller: %s of type"
                            " mpc_tracking_controller::MpcTrackingController",
                    plugin_name_.c_str());
        global_path_pub_->on_deactivate();
        local_path_pub_->on_deactivate();

    }

    void MpcTrackingController::setPlan(const nav_msgs::msg::Path & path)
    {
        this->global_plan_ = path;
    }

    bool MpcTrackingController::transformPose(
            const std::string frame,
            const geometry_msgs::msg::PoseStamped & in_pose,
            geometry_msgs::msg::PoseStamped & out_pose) const
    {
        if(in_pose.header.frame_id == frame)
        {
            out_pose = in_pose;
            return true;
        }

        try{
            //let tf do the transform of in_pose to target frame
            this->tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
            out_pose.header.frame_id = frame;
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger_,"Excptation in transformPose: %s", ex.what());
        }
        return false;
    }

    nav_msgs::msg::Path MpcTrackingController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose)
    {

        if (global_plan_.poses.empty()) {
            throw nav2_core::PlannerException("Received plan with zero length");
        }

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
            throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
        }

        // We'll discard points on the plan that are outside the local costmap
        nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
        const double max_costmap_dim = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
        const double max_transform_dist = max_costmap_dim * costmap->getResolution() / 2.0;

        // First find the closest pose on the path to the robot
        auto transformation_begin =
                nav2_util::geometry_utils::min_by(
                        global_plan_.poses.begin(), global_plan_.poses.end(),
                        [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
                            return euclidean_distance(robot_pose, ps);
                        });

        // Find points definitely outside of the costmap so we won't transform them.
        auto transformation_end = std::find_if(
                transformation_begin, end(global_plan_.poses),
                [&](const auto & global_plan_pose) {
                    return euclidean_distance(robot_pose, global_plan_pose) > max_transform_dist;
                });

        // Lambda to transform a PoseStamped from global frame to local
        auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
            geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
            stamped_pose.header.frame_id = global_plan_.header.frame_id;
            stamped_pose.header.stamp = robot_pose.header.stamp;
            stamped_pose.pose = global_plan_pose.pose;
            transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
            return transformed_pose;
        };

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan;
        std::transform(
                transformation_begin, transformation_end,
                std::back_inserter(transformed_plan.poses),
                transformGlobalPoseToLocal);
        transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
        transformed_plan.header.stamp = robot_pose.header.stamp; //std::transform

        // Remove the portion of the global plan that we've already passed so we don't
        // process it on the next iteration (this is called path pruning)
        global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
        global_path_pub_->publish(transformed_plan);

        if (transformed_plan.poses.empty()) {
            throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
        }

        return transformed_plan;
    }

    geometry_msgs::msg::TwistStamped MpcTrackingController::computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped & pose,
            const geometry_msgs::msg::Twist & velocity,
            nav2_core::GoalChecker* goal_checker )
    {

        

        geometry_msgs::msg::Pose pose_tolerance;
        geometry_msgs::msg::Twist vel_tolerance;
        if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
            RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
        } else {
            goal_dist_tol_ = pose_tolerance.position.x;
        }

        auto transformed_plan = transformGlobalPlan(pose);
        // if (this->global_plan_.poses.size() < 15) return do_pure_pusuit_tracking(pose, velocity , transformed_plan);
        if(0)
        {
            std::cout <<" pose frame: " <<pose.header.frame_id<<std::endl;
            std::cout <<" plan frame: " <<transformed_plan.header.frame_id<<std::endl;
            std::cout <<" plan size : " <<transformed_plan.poses.size()<<std::endl;
            std::cout <<" ----------------------------- " <<std::endl;
        }



        return do_mpc(pose,velocity,transformed_plan);
        // MPC result (all described in car frame), output = (acceleration, w)


    }



    geometry_msgs::msg::TwistStamped MpcTrackingController::do_mpc(const geometry_msgs::msg::PoseStamped & pose,
                          const geometry_msgs::msg::Twist & velocity , nav_msgs::msg::Path & transformed_plan )
    {
        vector<double> res;
        //mpc start

        //current state[x,y,theta,v]
        const double px = pose.pose.position.x;
        const double py = pose.pose.position.y;
        const double theta = tf2::getYaw(pose.pose.orientation);
        const double v = velocity.linear.x;

        //courrent control[a,w]
        const double w = _w;
        const double throttle = _throttle;
        const double dt = _dt;

        //Update path waypoints (conversion to odom frame)
        nav_msgs::msg::Path odom_path = nav_msgs::msg::Path();
        try{
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {
                double dx = transformed_plan.poses[1].pose.position.x - transformed_plan.poses[0].pose.position.x;
                double dy = transformed_plan.poses[1].pose.position.y - transformed_plan.poses[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }

            // Cut and downsampling the path
            for(unsigned long i =0; i< transformed_plan.poses.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {
                    geometry_msgs::msg::PoseStamped tempPose;


                    tf2_ros::TransformListener tfListener(*tf_);
                    geometry_msgs::msg::TransformStamped odom_transform;
                    odom_transform = tf_->lookupTransform(pose.header.frame_id,transformed_plan.header.frame_id,clock_->now(),transform_tolerance_);
//                    odom_transform = tf_->lookupTransform(_odom_frame, _map_frame, ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(transformed_plan.poses[i], tempPose, odom_transform); // robot_pose is the PoseStamp
                    odom_path.poses.push_back(tempPose);
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist;
                sampling = sampling + 1;
            }

            if(odom_path.poses.size() < 3)
            {
                RCLCPP_INFO(logger_,"Failed to path generation since small down-sampling path.");
                _waypointsDist = -1;
               return do_pure_pusuit_tracking(pose,velocity,transformed_plan);
            }


        }
        catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger_,"%s",ex.what());
        }


        if(0)
        {
            std::cout <<" pose frame: " <<pose.header.frame_id<<std::endl;
            std::cout <<" odom_path frame: " <<odom_path.poses[0].header.frame_id<<std::endl;
            std::cout <<" plan size : " <<transformed_plan.poses.size()<<std::endl;
            std::cout <<" ----------------------------- " <<std::endl;

        }


        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        // Convert to the vehicle coordinate system
        Eigen::VectorXd x_veh(N);
        Eigen::VectorXd y_veh(N);
        for(int i = 0; i < N; i++)
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
            //cout << "x_veh : " << x_veh[i]<< ", y_veh: " << y_veh[i] << endl;
        }
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3);
        const double cte  = polyeval(coeffs, 0.0);
        if(0)
        {
            cout << "coeffs : " << coeffs[0] << endl;
            cout << "pow : " << pow(0.0 ,0) << endl;
            cout << "cte : " << cte << endl;
        }

        double etheta = atan(coeffs[1]);

        // Global coordinate system about theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++)
        {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        double PI = 3.141592;

        // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity
        if(temp_theta <= -PI + traj_deg)
            temp_theta = temp_theta + 2 * PI;

        // Implementation about theta error more precisly
        if(gx && gy && temp_theta - traj_deg < 1.8 * PI)
            etheta = temp_theta - traj_deg;
        else
            etheta = 0;


        // Difference bewteen current position and goal position
//        const double x_err = goal_pose.pose.position.x -  base_odom.pose.pose.position.x;
//        const double y_err = goal_pose.pose.position.y -  base_odom.pose.pose.position.y;
//        const double goal_err = sqrt(x_err*x_err + y_err*y_err);


        Eigen::VectorXd state(6);
        if(0)
        {
            // Kinematic model is used to predict vehicle state at the actual moment of control (current time + delay dt)
            const double px_act = v * dt;
            const double py_act = 0;
            const double theta_act = w * dt; //(steering) theta_act = v * steering * dt / Lf;
            const double v_act = v + throttle * dt; //v = v + a * dt

            const double cte_act = cte + v * sin(etheta) * dt;
            const double etheta_act = etheta - theta_act;

            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }

        // Solve MPC Problem
//        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);
//        ros::Time end = ros::Time::now();
//        cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;

        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration

        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

//            if(0)
//            {
//                cout << "\n\nDEBUG" << endl;
//                cout << "theta: " << theta << endl;
//                cout << "V: " << v << endl;
//                //cout << "odom_path: \n" << odom_path << endl;
//                //cout << "x_points: \n" << x_veh << endl;
//                //cout << "y_points: \n" << y_veh << endl;
//                cout << "coeffs: \n" << coeffs << endl;
//                cout << "_w: \n" << _w << endl;
//                cout << "_throttle: \n" << _throttle << endl;
//                cout << "_speed: \n" << _speed << endl;
//            }

        // TODO:Display the MPC predicted trajectory
        nav_msgs::msg::Path local_path;
        local_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        local_path.header.stamp = pose.header.stamp;

        tf2::Quaternion myQuaternion;

        for(unsigned long i=0; i<_mpc.mpc_x.size(); i++)
        {
            geometry_msgs::msg::PoseStamped tempPose;
            tempPose.header = local_path.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];

            local_path.poses.push_back(tempPose);
        }
        local_path_pub_->publish(local_path);

        geometry_msgs::msg::TwistStamped cmdvel;
        cmdvel.twist.linear.x = _speed;
        cmdvel.twist.angular.z = _w;
        cmdvel.header.frame_id = pose.header.frame_id;
        cmdvel.header.stamp = this->clock_->now();

        return cmdvel;




//        return mpc_results;
    }




    // Evaluate a polynomial.
    double MpcTrackingController::polyeval(Eigen::VectorXd coeffs, double x)
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }



    // Fit a polynomial.
    // Adapted from
    // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
    Eigen::VectorXd MpcTrackingController::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++)
        {
            for (int i = 0; i < order; i++)
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void MpcTrackingController::setSpeedLimit(const double & speed_limit, const bool & percentage)
    {
        if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
            // Restore default value
            _max_speed = 0.5;
        } else {
            if (percentage) {
                // Speed limit is expressed in % from maximum speed of robot
                _max_speed = 0.5 * speed_limit / 100.0;
            } else {
                // Speed limit is expressed in absolute value
                _max_speed = speed_limit;
            }
        }

    }



} //namespace

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
        mpc_tracking_controller::MpcTrackingController,
        nav2_core::Controller)