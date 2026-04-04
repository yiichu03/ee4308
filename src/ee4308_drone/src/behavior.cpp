#include "ee4308_drone/behavior.hpp"

namespace
{
    const char *getBehaviorStateName(const int state)
    {
        switch (state)
        {
        case 0:
            return "BEGIN";
        case 1:
            return "TAKEOFF";
        case 2:
            return "TURTLE_POSITION";
        case 3:
            return "TURTLE_WAYPOINT";
        case 4:
            return "INITIAL";
        case 5:
            return "LANDING";
        case 6:
            return "END";
        default:
            return "UNKNOWN";
        }
    }
}

namespace ee4308::drone
{

    Behavior::Behavior(
        const rclcpp::NodeOptions &options,
        const std::string &name = "behavior")
        : Node(name, options)
    {
        // parameters
        this->reached_thres_ = ee4308::getParameter<double>(this, "reached_thres", 0.2).as_double();
        this->cruise_height_ = ee4308::getParameter<double>(this, "cruise_height", 4.0).as_double();
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 5.0).as_double();
        this->topic_turtle_plan_ = ee4308::getParameter<std::string>(this, "topic_turtle_plan", "/turtle/plan").as_string();
        this->topic_turtle_stop_ = ee4308::getParameter<std::string>(this, "topic_turtle_stop", "/turtle/stop").as_string();
        this->frame_id_map_ = ee4308::getParameter<std::string>(this, "map_frame_id", "map").as_string();
        this->initial_x_ = ee4308::getParameter<double>(this, "initial_x", -2.0).as_double();
        this->initial_y_ = ee4308::getParameter<double>(this, "initial_y", -2.0).as_double();
        this->initial_z_ = ee4308::getParameter<double>(this, "initial_z", 0.05).as_double();

        // topics
        this->sub_est_pose_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
                                                                                 std::bind(&Behavior::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_turtle_stop_ = this->create_subscription<std_msgs::msg::Empty>(
            this->topic_turtle_stop_, rclcpp::ServicesQoS(),
            std::bind(&Behavior::callbackSubTurtleStop_, this, std::placeholders::_1));
        this->sub_turtle_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            this->topic_turtle_plan_, rclcpp::SensorDataQoS(),
            std::bind(&Behavior::callbackSubTurtlePlan_, this, std::placeholders::_1));
        this->pub_enable_ = this->create_publisher<std_msgs::msg::Bool>("enable", rclcpp::ServicesQoS());

        // services
        this->cli_plan_ = this->create_client<nav_msgs::srv::GetPlan>("get_plan");

        // states
        this->turtle_stop_ = false;
        this->plan_requested_ = false;
        this->state_ = BEGIN;
        this->transition_(TAKEOFF);
        this->timer_ = this->create_timer(1s / frequency_, std::bind(&Behavior::callbackTimer_, this));
    }

    void Behavior::callbackSubOdom_(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->odom_ = *msg;
    }

    void Behavior::callbackSubTurtleStop_(std_msgs::msg::Empty::SharedPtr msg)
    {
        (void)msg;
        this->turtle_stop_ = true;
    }

    void Behavior::callbackSubTurtlePlan_(nav_msgs::msg::Path::SharedPtr msg)
    {
        this->turtle_plan_ = *msg;
    }

    void Behavior::callbackTimer_()
    {
        // Continuously update waypoint for TURTLE_POSITION to chase the moving turtle
        if (state_ == TURTLE_POSITION && !turtle_plan_.poses.empty())
        {
            waypoint_x_ = turtle_plan_.poses.front().pose.position.x;
            waypoint_y_ = turtle_plan_.poses.front().pose.position.y;
            waypoint_z_ = cruise_height_;
        }

        if (reachedWaypoint_())
        {
            if (state_ == TAKEOFF)
            {
                // After reaching initial air position, start cycle or handle early turtle stop
                if (turtle_stop_)
                    transition_(INITIAL);
                else
                    transition_(TURTLE_POSITION);
            }
            else if (state_ == INITIAL)
            {
                // At initial air position: land if turtle done, else start next cycle
                if (turtle_stop_)
                    transition_(LANDING);
                else
                    transition_(TURTLE_POSITION);
            }
            else if (state_ == TURTLE_POSITION)
            {
                transition_(TURTLE_WAYPOINT);
            }
            else if (state_ == TURTLE_WAYPOINT)
            {
                transition_(INITIAL);
            }
            else if (state_ == LANDING)
            {
                transition_(END);
            }
        }

        // Request a plan every timer tick so the controller always has a path
        requestPlan_(
            odom_.pose.pose.position.x,
            odom_.pose.pose.position.y,
            odom_.pose.pose.position.z,
            waypoint_x_, waypoint_y_, waypoint_z_);
    }

    void Behavior::transition_(int new_state)
    {
        // TMP LOG: remove before submission after behavior/controller integration is stable.
        RCLCPP_INFO(
            this->get_logger(),
            "Behavior transition %s -> %s | turtle_stop=%s | waypoint=(%.2f, %.2f, %.2f)",
            getBehaviorStateName(state_),
            getBehaviorStateName(new_state),
            turtle_stop_ ? "true" : "false",
            waypoint_x_,
            waypoint_y_,
            waypoint_z_);

        state_ = new_state;

        if (state_ == TAKEOFF)
        {
            // Turn on the robot and fly to initial air position
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = true;
            this->pub_enable_->publish(msg_enable);
            setWaypoint_(initial_x_, initial_y_, cruise_height_);
        }
        else if (state_ == INITIAL)
        {
            // Return to initial air position
            setWaypoint_(initial_x_, initial_y_, cruise_height_);
        }
        else if (state_ == TURTLE_POSITION)
        {
            // Fly to turtle's current position (front of its path) at cruise height
            if (!turtle_plan_.poses.empty())
            {
                setWaypoint_(
                    turtle_plan_.poses.front().pose.position.x,
                    turtle_plan_.poses.front().pose.position.y,
                    cruise_height_);
            }
        }
        else if (state_ == TURTLE_WAYPOINT)
        {
            // Fly to turtle's current goal (back of its path) at cruise height
            if (!turtle_plan_.poses.empty())
            {
                setWaypoint_(
                    turtle_plan_.poses.back().pose.position.x,
                    turtle_plan_.poses.back().pose.position.y,
                    cruise_height_);
            }
        }
        else if (state_ == LANDING)
        {
            // Descend to initial ground position
            setWaypoint_(initial_x_, initial_y_, initial_z_);
        }
        else if (state_ == END)
        {
            // Turn off the robot and stop the state machine
            std_msgs::msg::Bool msg_enable;
            msg_enable.data = false;
            this->pub_enable_->publish(msg_enable);
            this->timer_->cancel();
            this->timer_ = nullptr;
        }

        // TMP LOG: remove before submission after behavior/controller integration is stable.
        RCLCPP_INFO(
            this->get_logger(),
            "Behavior target %s | new_waypoint=(%.2f, %.2f, %.2f)",
            getBehaviorStateName(state_),
            waypoint_x_,
            waypoint_y_,
            waypoint_z_);
    }

    void Behavior::setWaypoint_(double waypoint_x, double waypoint_y, double waypoint_z)
    {
        waypoint_x_ = waypoint_x;
        waypoint_y_ = waypoint_y;
        waypoint_z_ = waypoint_z;
    }

    bool Behavior::reachedWaypoint_()
    {
        double dx = odom_.pose.pose.position.x - waypoint_x_;
        double dy = odom_.pose.pose.position.y - waypoint_y_;
        double dz = odom_.pose.pose.position.z - waypoint_z_;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        return dist < reached_thres_;
    }

    void Behavior::requestPlan_(double drone_x, double drone_y, double drone_z,
                                double waypoint_x, double waypoint_y, double waypoint_z)
    { // Sends a non-blocking service request to publish a path from the planner node.
        if (plan_requested_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "No request is made as there is no response yet from previous request.");
            return;
        }
        plan_requested_ = true;
        auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
        request->goal.header.frame_id = this->frame_id_map_;
        request->goal.header.stamp = this->now();
        request->goal.pose.position.x = waypoint_x;
        request->goal.pose.position.y = waypoint_y;
        request->goal.pose.position.z = waypoint_z;
        request->start.header.frame_id = this->frame_id_map_;
        request->start.header.stamp = this->now();
        request->start.pose.position.x = drone_x;
        request->start.pose.position.y = drone_y;
        request->start.pose.position.z = drone_z;
        request_plan_future_ = cli_plan_->async_send_request(request, std::bind(&Behavior::callbackCliReceivePlan_, this, std::placeholders::_1)).future;
    }

    void Behavior::callbackCliReceivePlan_(const rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
    {
        (void)future;
        plan_requested_ = false;
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Behavior);
