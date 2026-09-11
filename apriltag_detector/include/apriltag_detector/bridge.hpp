#pragma once

#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace vision_tools
{
  class AprilTagBridge : public rclcpp::Node
  {
  public:
    AprilTagBridge(const rclcpp::NodeOptions &options);
    ~AprilTagBridge() {};

  private:
    rclcpp::Subscription<extender_msgs::msg::SharedControlGoalArray>::SharedPtr apriltag_sub_;
    rclcpp::Publisher<extender_msgs::msg::SharedControlGoalArray>::SharedPtr goal_pub_;

    std::string target_frame_;
    std::string base_frame_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void tagCallback(const extender_msgs::msg::SharedControlGoalArray::SharedPtr msg);
  };

} // namespace vision_tools