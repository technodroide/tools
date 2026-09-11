#include "apriltag_detector/bridge.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vision_tools
{
  AprilTagBridge::AprilTagBridge(const rclcpp::NodeOptions &options)
      : Node("apriltag_bridge", options)
  {
    // Declare and get parameters
    this->declare_parameter("target_frame", "base_link");
    target_frame_ = this->get_parameter("target_frame").as_string();
    this->declare_parameter("base_frame", "camera_link");               // add to linked "base_frame" and "camera_link"   R.G 20260710
    base_frame_ = this->get_parameter("base_frame").as_string();

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    apriltag_sub_ = this->create_subscription<extender_msgs::msg::SharedControlGoalArray>(
        "/tag_detections", 10,
        std::bind(&AprilTagBridge::tagCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<extender_msgs::msg::SharedControlGoalArray>(
        "/shared_control/dynamic_goals", 10);

    RCLCPP_INFO(this->get_logger(), "AprilTag Bridge started. Transforming to: %s",
                target_frame_.c_str());
  }

  void AprilTagBridge::tagCallback(const extender_msgs::msg::SharedControlGoalArray::SharedPtr msg)
  {

    if (msg->goal_array.empty())
    {
      return;
    }
    extender_msgs::msg::SharedControlGoalArray out_msg;
    out_msg.header.stamp = msg->header.stamp;
    out_msg.header.frame_id = target_frame_;
    try
    {
      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(target_frame_, base_frame_, msg->header.stamp,
                                      rclcpp::Duration::from_seconds(0.1));

      for (const auto &tag : msg->goal_array)
      {
        extender_msgs::msg::SharedControlGoal goal_msg;
        geometry_msgs::msg::Pose transformed_pose;

        tf2::doTransform(tag.goal_pose, transformed_pose, transform);

        goal_msg.id = tag.id;
        goal_msg.goal_pose = transformed_pose;

        out_msg.goal_array.push_back(goal_msg);
      }

      goal_pub_->publish(out_msg);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not transform goal: %s", ex.what());
    }
  }
} // namespace vision_tools

RCLCPP_COMPONENTS_REGISTER_NODE(vision_tools::AprilTagBridge)