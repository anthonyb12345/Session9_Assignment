#ifndef LAP_TIME_CLIENT_HPP
#define LAP_TIME_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "turtlebot3_interfaces/action/measure_lap_time.hpp"
#include "turtlebot3_interfaces/srv/lap_starting_time.hpp"

class LapTimeActionClient : public rclcpp::Node
{
public:
  using MeasureLapTime = turtlebot3_interfaces::action::MeasureLapTime;
  using GoalHandleMeasureLapTime = rclcpp_action::ClientGoalHandle<MeasureLapTime>;
  using LapStartingTime = turtlebot3_interfaces::srv::LapStartingTime;

  explicit LapTimeActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Client<MeasureLapTime>::SharedPtr action_client_;
  rclcpp::Client<turtlebot3_interfaces::srv::LapStartingTime>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_goal();
  

  void goal_response_callback(const GoalHandleMeasureLapTime::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleMeasureLapTime::SharedPtr, const std::shared_ptr<const MeasureLapTime::Feedback> feedback);
  void result_callback(const GoalHandleMeasureLapTime::WrappedResult & result);

 
};

#endif // LAP_TIME_CLIENT_HPP










