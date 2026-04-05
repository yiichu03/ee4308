#include "ee4308_drone/controller.hpp"

namespace ee4308::drone
{
    Controller::Controller(
        const rclcpp::NodeOptions &options,
        const std::string &name = "controller") 
        : Node(name, options)
    {
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 20.0).as_double();
        this->enable_ = ee4308::getParameter<bool>(this, "enable", true).as_bool();
        this->lookahead_distance_ = ee4308::getParameter<double>(this, "lookahead_distance", 1.0).as_double();
        this->max_xy_vel_ = ee4308::getParameter<double>(this, "max_xy_vel", 1.0).as_double();
        this->max_z_vel_ = ee4308::getParameter<double>(this, "max_z_vel", 0.5).as_double();
        this->yaw_vel_ = ee4308::getParameter<double>(this, "yaw_vel", 0.3).as_double();
        this->kp_xy_ = ee4308::getParameter<double>(this, "kp_xy", 1.0).as_double();
        this->kp_z_ = ee4308::getParameter<double>(this, "kp_z", 1.0).as_double();

        this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubPlan_, this, std::placeholders::_1));

        this->received_odom_ = false;

        this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
    }

    void Controller::callbackSubOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->odom_ = msg;
        this->received_odom_ = true;
    }

    void Controller::callbackSubPlan_(const nav_msgs::msg::Path msg)
    {
        this->plan_ = msg;
    }

    void Controller::callbackTimer_()
    {
        if (!enable_)
            return;

        if (!this->received_odom_)
        {
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        // 无人机当前位置和朝向（yaw）
        double drone_x = odom_.pose.pose.position.x;
        double drone_y = odom_.pose.pose.position.y;
        double drone_z = odom_.pose.pose.position.z;
        double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);

        // 1. 找路径上距无人机最近的点
        int closest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - drone_x;
            double dy = plan_.poses[i].pose.position.y - drone_y;
            double dz = plan_.poses[i].pose.position.z - drone_z;
            double dist = std::hypot(dx, std::hypot(dy, dz));
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_idx = i;
            }
        }

        // 2. 从最近点向前找 lookahead 点（超过 lookahead_distance_ 或到达终点）
        int lookahead_idx = (int)plan_.poses.size() - 1;
        for (int i = closest_idx; i < (int)plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - drone_x;
            double dy = plan_.poses[i].pose.position.y - drone_y;
            double dz = plan_.poses[i].pose.position.z - drone_z;
            double dist = std::hypot(dx, std::hypot(dy, dz));
            lookahead_idx = i;
            if (dist >= lookahead_distance_)
                break;
        }

        double lx = plan_.poses[lookahead_idx].pose.position.x;
        double ly = plan_.poses[lookahead_idx].pose.position.y;
        double lz = plan_.poses[lookahead_idx].pose.position.z;

        // 3. 世界坐标系下的速度（比例控制）
        double vx_world = kp_xy_ * (lx - drone_x);
        double vy_world = kp_xy_ * (ly - drone_y);
        double vz      = kp_z_  * (lz - drone_z);

        // 4. 限制水平速度大小（保持方向，只缩放幅值）
        double xy_speed = std::hypot(vx_world, vy_world);
        if (xy_speed > max_xy_vel_)
        {
            vx_world = vx_world / xy_speed * max_xy_vel_;
            vy_world = vy_world / xy_speed * max_xy_vel_;
        }

        // 5. 限制垂直速度
        vz = std::clamp(vz, -max_z_vel_, max_z_vel_);

        // 6. 世界坐标系 → 无人机 body frame（按 yaw 旋转）
        double vx_body =  vx_world * std::cos(yaw) + vy_world * std::sin(yaw);
        double vy_body = -vx_world * std::sin(yaw) + vy_world * std::cos(yaw);

        // 7. 发布速度指令，yaw 始终按 yaw_vel_ 旋转
        publishCmdVel_(vx_body, vy_body, vz, yaw_vel_);
    }

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(z_vel) || !std::isfinite(yaw_vel))
        {
            RCLCPP_WARN(this->get_logger(), 
                "Cmd velocities are inf or nan. Controller or estimator problem. CmdVels(x,y,z,yaw): %6.3f, %6.3f, %6.3f, %6.3f", 
                x_vel, y_vel, z_vel, yaw_vel);
        }
        pub_cmd_vel_->publish(cmd_vel);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Controller);
