#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class WaypointFollower : public rclcpp::Node {
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleJT = rclcpp_action::ClientGoalHandle<FollowJT>;

  WaypointFollower() : Node("waypoint_follower") {
    client_ = rclcpp_action::create_client<FollowJT>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");

    if (!client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available!");
      rclcpp::shutdown();
    }

    send_waypoints();
  }

private:
  rclcpp_action::Client<FollowJT>::SharedPtr client_;

  void send_waypoints() {
    std::vector<std::vector<double>> waypoints = {
        {0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8},
        {0.3, -0.3, 0.0, -1.5, 0.0, 1.2, 0.5},
        {-0.3, -0.6, 0.0, -2.2, 0.0, 1.8, 1.0}};

    int idx = 0;
    for (const auto &wp : waypoints) {
      auto goal_msg = FollowJT::Goal();
      goal_msg.trajectory.joint_names = {
          "panda_joint1", "panda_joint2", "panda_joint3",
          "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = wp;
      point.time_from_start = rclcpp::Duration::from_seconds(5.0); // 5 sec move
      goal_msg.trajectory.points.push_back(point);

      RCLCPP_INFO(this->get_logger(), "Sending waypoint %d", idx++);

      auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
      send_goal_options.result_callback = [this](const GoalHandleJT::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Waypoint executed successfully.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to execute waypoint.");
        }
      };

      auto future_goal = client_->async_send_goal(goal_msg, send_goal_options);
      auto goal_handle = future_goal.get();

      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
        continue;
      }

      auto future_result = client_->async_get_result(goal_handle);
      auto result = future_result.get();  // <-- waits until trajectory finishes
    }

    RCLCPP_INFO(this->get_logger(), "All waypoints executed.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
