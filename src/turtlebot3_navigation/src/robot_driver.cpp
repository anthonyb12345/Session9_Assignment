#include "robot_driver.hpp"
#include <algorithm>
#include <chrono>


using namespace std::chrono_literals;

RobotDriver::RobotDriver() 
: Node("robot_driver"),
  closest_distance_(std::numeric_limits<float>::infinity()), 
  closest_angle_(0.0), 
  front_distance_(0.0), 
  left_distance_(0.0),
  stop_thread_(false),
  node_lap_client_ready(false) {

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto scan_sub_opt = rclcpp::SubscriptionOptions();
    scan_sub_opt.callback_group = callback_group_;

    client_ = this->create_client<turtlebot3_interfaces::srv::FindClosestWall>("find_closest_wall", rmw_qos_profile_services_default, callback_group_);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&RobotDriver::scan_callback, this, std::placeholders::_1), scan_sub_opt);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    lap_starting_time_service_ = this->create_service<turtlebot3_interfaces::srv::LapStartingTime>(
        "lap_starting_time",
        std::bind(&RobotDriver::handle_lap_starting_time, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "RobotDriver node started...");

    auto request = std::make_shared<turtlebot3_interfaces::srv::FindClosestWall::Request>();


    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully found the closest wall.");
            follow_wall_thread_ = std::thread([this]() { this->follow_wall(); });
        } else {
            RCLCPP_INFO(this->get_logger(), "Failed to find the closest wall.");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service find_closest_wall");
    }
}
void RobotDriver::handle_lap_starting_time(const std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime::Request> /*request*/,
                                           std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime::Response> response) {
    if (node_lap_client_ready == true){
        response->success = true;
    }else{
        response->success = false;
    }    
}

void RobotDriver::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto ranges = msg->ranges;
    closest_distance_ = *std::min_element(ranges.begin(), ranges.end());
    auto min_index = std::distance(ranges.begin(), std::min_element(ranges.begin(), ranges.end()));
    closest_angle_ = msg->angle_min + min_index * msg->angle_increment;
    front_distance_ = msg->ranges[0];
    left_distance_ = ranges[ranges.size() / 4];  // 90 degrees to the left
    front_index = msg->angle_min;
    left_index =  msg->angle_min + 1.5708;
    //RCLCPP_INFO(this->get_logger(), "front_distance_: %f, closest_angle_: %f, left_distance_: %f", front_distance_, closest_angle_, left_distance_);
}

void RobotDriver::follow_wall() {
    //RCLCPP_INFO(this->get_logger(), "Following Wall");
    //RCLCPP_INFO(this->get_logger(), "front_distance_: %f, closest_angle_: %f, left_distance_: %f", front_distance_, closest_angle_, left_distance_);
    
    
    auto twist = geometry_msgs::msg::Twist();
    while(true){
        if (front_distance_ < left_distance_) {
            while (std::abs(front_index - closest_angle_) < 1.5708) {
                //RCLCPP_INFO(this->get_logger(), "abs: %f", std::abs(front_index- closest_angle_));
                twist.angular.z = -0.2;
                publisher_->publish(twist);
            

                
            }
        } else if (front_distance_ > left_distance_) {
            while (std::abs(left_index - closest_angle_) < 1.5708) {
                twist.angular.z = -0.2;  
                publisher_->publish(twist);
                
            }
        }
        node_lap_client_ready = true;
        twist.angular.z = 0.0;
        publisher_->publish(twist);

        while (front_distance_ > 0.5) {
            twist.linear.x = 0.2;
            publisher_->publish(twist);
            if (left_distance_ < 0.45){
                twist.angular.z = -0.01;
                publisher_->publish(twist);
            }
            if (left_distance_ > 0.55){
                twist.angular.z = 0.01;
                publisher_->publish(twist);
            }
            
        }

        twist.linear.x = 0.0;
        publisher_->publish(twist);
        
    }
}
RobotDriver::~RobotDriver() {
    stop_thread_ = true;
    if (follow_wall_thread_.joinable()) {
        follow_wall_thread_.join();
    }
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



