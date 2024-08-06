#ifndef ROBOT_DRIVER_HPP_
#define ROBOT_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_interfaces/srv/find_closest_wall.hpp"
#include "turtlebot3_interfaces/srv/lap_starting_time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <limits>
#include <memory>
#include <thread>


class RobotDriver : public rclcpp::Node {
public:
    RobotDriver();
    ~RobotDriver();

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void follow_wall();
    void handle_lap_starting_time(const std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime::Request> request,
                                  std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime::Response> response);


    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<turtlebot3_interfaces::srv::FindClosestWall>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
     rclcpp::Service<turtlebot3_interfaces::srv::LapStartingTime>::SharedPtr lap_starting_time_service_;

    float closest_distance_;
    float closest_angle_;
    float front_distance_;
    float left_distance_;
    float front_index;
    float left_index;
    std::thread follow_wall_thread_;
    std::atomic<bool> stop_thread_;
    bool node_lap_client_ready;

};

#endif  // ROBOT_DRIVER_HPP_

