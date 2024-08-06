#include "lap_time_client.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>

using namespace std::chrono_literals;
using MeasureLapTime = turtlebot3_interfaces::action::MeasureLapTime;
using LapStartingTime = turtlebot3_interfaces::srv::LapStartingTime;

LapTimeActionClient::LapTimeActionClient(const rclcpp::NodeOptions & options)
: Node("lap_time_action_client", options)
{
  action_client_ = rclcpp_action::create_client<MeasureLapTime>(this, "lap_time");
  service_client_ = this->create_client<turtlebot3_interfaces::srv::LapStartingTime>("lap_starting_time", rmw_qos_profile_services_default);
 
  bool success = false;
  while (rclcpp::ok() && !success) {
    auto request = std::make_shared<LapStartingTime::Request>();

    // Wait for the service to be available
    while (!service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = service_client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Lap Starting...");
        success = true;
        this->send_goal();
        
      } else {
        //RCLCPP_INFO(this->get_logger(), "Waiting to start lap, retrying...");
        std::this_thread::sleep_for(1s);
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service lap_starting_time, retrying...");
      std::this_thread::sleep_for(1s);
    }
  }
  

  
}



void LapTimeActionClient::send_goal()
{
  using namespace std::placeholders;

  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = MeasureLapTime::Goal();
  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<MeasureLapTime>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&LapTimeActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&LapTimeActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&LapTimeActionClient::result_callback, this, _1);
  this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeActionClient::goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void LapTimeActionClient::feedback_callback(
  GoalHandleMeasureLapTime::SharedPtr,
  const std::shared_ptr<const MeasureLapTime::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds", feedback->elapsed_time);
}

void LapTimeActionClient::result_callback(const GoalHandleMeasureLapTime::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Lap completed in %f seconds", result.result->total_time);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<LapTimeActionClient>();
  rclcpp::spin(action_client);
  rclcpp::shutdown();
  return 0;
}












