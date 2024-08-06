#include "lap_time_server.hpp"

using namespace std::chrono_literals;

LapTimeActionServer::LapTimeActionServer()
: Node("lap_time_action_server"),
  start_position_{},
  current_position_{},
  start_time_()
{
    action_server_ = rclcpp_action::create_server<MeasureLapTime>(
        this,
        "lap_time",
        std::bind(&LapTimeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LapTimeActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&LapTimeActionServer::handle_accepted, this, std::placeholders::_1)
    );
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(&LapTimeActionServer::odom_callback, this, std::placeholders::_1)
    );
}

void LapTimeActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_position_ = msg->pose.pose;
    // RCLCPP_INFO(this->get_logger(), "Current position: [%f, %f, %f]", current_position_.position.x, current_position_.position.y, current_position_.position.z);
}

rclcpp_action::GoalResponse
LapTimeActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const MeasureLapTime::Goal> /*goal*/
)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
LapTimeActionServer::handle_cancel(const std::shared_ptr<GoalHandleLapTime> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeActionServer::handle_accepted(const std::shared_ptr<GoalHandleLapTime> goal_handle)
{
    std::thread(
        [this, goal_handle]() {
            this->execute(goal_handle);
        }
    ).detach();
}

void LapTimeActionServer::execute(const std::shared_ptr<GoalHandleLapTime> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Starting lap timer...");

    auto goal = goal_handle->get_goal();
    start_time_ = this->get_clock()->now();
    start_position_ = current_position_;

    while (rclcpp::ok() && !this->is_lap_completed())
    {
        auto elapsed_time = (this->get_clock()->now() - start_time_).seconds();
        auto feedback_msg = std::make_shared<MeasureLapTime::Feedback>();
        feedback_msg->elapsed_time = elapsed_time;
        goal_handle->publish_feedback(feedback_msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto total_time = (this->get_clock()->now() - start_time_).seconds();
    auto result_msg = std::make_shared<MeasureLapTime::Result>();
    result_msg->total_time = total_time;

    if (rclcpp::ok())
    {
        goal_handle->succeed(result_msg);
        RCLCPP_INFO(this->get_logger(), "Lap completed in %f seconds", total_time);
    }
    else
    {
        goal_handle->abort(result_msg);
        RCLCPP_ERROR(this->get_logger(), "Lap failed");
    }
}

bool LapTimeActionServer::is_lap_completed()
{
    if ((this->get_clock()->now() - start_time_).seconds() > 20.0)
    {
        double distance = calculate_distance(start_position_, current_position_);
        if (distance < 0.3)
        {
            return true;
        }
    }
    return false;
}

double LapTimeActionServer::calculate_distance(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &current)
{
    double dx = current.position.x - start.position.x;
    double dy = current.position.y - start.position.y;
    double dz = current.position.z - start.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LapTimeActionServer>());
    rclcpp::shutdown();
    return 0;
}






