#include "wall_finder_server.hpp"

WallFinderServer::WallFinderServer()
    : Node("wall_finder_server"),
      closest_distance_(std::numeric_limits<float>::infinity()),
      closest_angle_(0.0),
      first_index_(0.0) {
    
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    auto scan_sub_opt = rclcpp::SubscriptionOptions();
    scan_sub_opt.callback_group = callback_group_;
   
    srv_ = this->create_service<turtlebot3_interfaces::srv::FindClosestWall>(
        "find_closest_wall",
        std::bind(&WallFinderServer::find_closest_wall_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&WallFinderServer::scan_callback, this, std::placeholders::_1),
        scan_sub_opt
    );
}

void WallFinderServer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    

    auto ranges = msg->ranges;
    auto min_it = std::min_element(ranges.begin(), ranges.end());
    closest_distance_ = *min_it;
    auto min_index = std::distance(ranges.begin(), min_it);
    closest_angle_ = msg->angle_min + min_index * msg->angle_increment;
    first_index_ = msg->angle_min;
    // RCLCPP_INFO(this->get_logger(), "Closest wall at distance: %f, angle: %f", closest_distance_, closest_angle_);
}

void WallFinderServer::find_closest_wall_callback(
    const std::shared_ptr<turtlebot3_interfaces::srv::FindClosestWall::Request> /* request */,
    std::shared_ptr<turtlebot3_interfaces::srv::FindClosestWall::Response> response) {

    auto start = std::chrono::steady_clock::now();
    while (closest_angle_ == 0.0 && first_index_ == 0.0) {
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Finding the closest wall...");
    auto twist = geometry_msgs::msg::Twist();

    std::this_thread::sleep_for(7s);

    twist.linear.x = 0.2;
    publisher_->publish(twist);
    std::this_thread::sleep_for(1s);
    twist.linear.x = 0.0;
    publisher_->publish(twist);

    while (std::abs(first_index_ - closest_angle_) > 0.1) {
        // RCLCPP_INFO(this->get_logger(), "first index - closest angle = %f", first_index_ - closest_angle_);
        twist.angular.z = 0.2;
        publisher_->publish(twist);
    }

    twist.angular.z = 0.0;
    publisher_->publish(twist);

    while (closest_distance_ > 0.5) {
        twist.linear.x = 0.2;
        publisher_->publish(twist);
    }

    twist.linear.x = 0.0;
    publisher_->publish(twist);

    RCLCPP_INFO(this->get_logger(), "Wall found and robot stopped.");
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    RCLCPP_INFO(this->get_logger(), "Method find_closest_wall_callback took %ld milliseconds to execute.", duration.count());
    response->success = true;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFinderServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}

