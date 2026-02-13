#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        // initialize topics
        // this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
        //     "scan", rclcpp::SensorDataQoS(),
        //     std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    // void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    // {
    //     this->scan_ranges_ = msg->ranges;
    // }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();

        const double rbt_x = rbt_pose.pose.position.x;
        const double rbt_y = rbt_pose.pose.position.y;

        // stop when close enough to goal
        const double goal_dist = std::hypot(
            goal_pose.pose.position.x - rbt_x,
            goal_pose.pose.position.y - rbt_y);
        if (goal_dist < xy_goal_thres_)
            return writeCmdVel(0, 0);

        // find closest point on plan
        std::size_t closest_idx = 0;
        double min_dist = std::hypot(
            global_plan_.poses[0].pose.position.x - rbt_x,
            global_plan_.poses[0].pose.position.y - rbt_y);
        for (std::size_t i = 1; i < global_plan_.poses.size(); ++i)
        {
            const double dist = std::hypot(
                global_plan_.poses[i].pose.position.x - rbt_x,
                global_plan_.poses[i].pose.position.y - rbt_y);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // find first lookahead point at least desired_lookahead_dist_ away
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;
        for (std::size_t i = closest_idx; i < global_plan_.poses.size(); ++i)
        {
            const double dist = std::hypot(
                global_plan_.poses[i].pose.position.x - rbt_x,
                global_plan_.poses[i].pose.position.y - rbt_y);
            if (dist >= desired_lookahead_dist_)
            {
                lookahead_pose = global_plan_.poses[i];
                break;
            }
        }

        // transform lookahead point to robot frame
        const double rbt_yaw = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);
        const double dx = lookahead_pose.pose.position.x - rbt_x;
        const double dy = lookahead_pose.pose.position.y - rbt_y;
        const double x_rbt = dx * std::cos(rbt_yaw) + dy * std::sin(rbt_yaw);
        const double y_rbt = -dx * std::sin(rbt_yaw) + dy * std::cos(rbt_yaw);

        // pure pursuit curvature and command
        const double L2 = x_rbt * x_rbt + y_rbt * y_rbt;
        if (L2 <= ee4308::THRES)
            return writeCmdVel(0, 0);

        const double curvature = (2.0 * y_rbt) / L2;
        double linear_vel = desired_linear_vel_;
        double angular_vel = linear_vel * curvature;

        linear_vel = std::clamp(linear_vel, 0.0, max_linear_vel_);
        angular_vel = std::clamp(angular_vel, -max_angular_vel_, max_angular_vel_);

        return writeCmdVel(linear_vel, angular_vel);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // ======================================== DO NOT TOUCH =================================

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)
