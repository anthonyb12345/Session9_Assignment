#ifndef WALL_FINDER_SERVER_HPP
#define WALL_FINDER_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <turtlebot3_interfaces/srv/find_closest_wall.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include <limits>

using namespace std::chrono_literals;

class WallFinderServer : public rclcpp::Node {
public:
    WallFinderServer();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void find_closest_wall_callback(const std::shared_ptr<turtlebot3_interfaces::srv::FindClosestWall::Request> request,
                                    std::shared_ptr<turtlebot3_interfaces::srv::FindClosestWall::Response> response);

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<turtlebot3_interfaces::srv::FindClosestWall>::SharedPtr srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    float closest_distance_;
    float closest_angle_;
    float first_index_;
};

#endif  // WALL_FINDER_SERVER_HPP
