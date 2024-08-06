#ifndef LAP_TIME_SERVER_HPP
#define LAP_TIME_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlebot3_interfaces/action/measure_lap_time.hpp>

using MeasureLapTime = turtlebot3_interfaces::action::MeasureLapTime;
using GoalHandleLapTime = rclcpp_action::ServerGoalHandle<MeasureLapTime>;

class LapTimeActionServer : public rclcpp::Node
{
public:
    LapTimeActionServer();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MeasureLapTime::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLapTime> goal_handle
    );
    void handle_accepted(const std::shared_ptr<GoalHandleLapTime> goal_handle);
    void execute(const std::shared_ptr<GoalHandleLapTime> goal_handle);
    bool is_lap_completed();
    double calculate_distance(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &current);

    rclcpp_action::Server<MeasureLapTime>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
    geometry_msgs::msg::Pose start_position_;
    geometry_msgs::msg::Pose current_position_;
    rclcpp::Time start_time_;
};

#endif // LAP_TIME_SERVER_HPP


