#pragma once

#include <string>
#include <unordered_map>

#include "extender_msgs/msg/shared_control_goal.hpp"
#include "extender_msgs/msg/shared_control_goal_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/image_encodings.hpp>

#include <apriltag.h>
#include <apriltag_pose.h>
#include <tag36h11.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace vision_tools
{
  class AprilTagDetector : public rclcpp::Node
  {
  public:
    AprilTagDetector(const rclcpp::NodeOptions &options);
    ~AprilTagDetector();

  private:
    apriltag_family_t *tf_ = nullptr;
    apriltag_detector_t *td_ = nullptr;
    zarray_t *detections_ = nullptr;
    apriltag_detection_t *det = nullptr;
    apriltag_detection_info_t info;
    cv::VideoCapture cap_;

    std::unordered_map<int, double> tag_sizes_;
    int max_hamming_distance_;

    cv::Mat current_frame_;

    bool has_camera_info_ = false;
    double fx_, fy_, cx_, cy_;

    rclcpp::Publisher<extender_msgs::msg::SharedControlGoalArray>::SharedPtr tag_publisher_;
    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo msg);

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      // USB_cam flux
    void imageDirectCallback(
      cv::VideoCapture &cap,
      //const sensor_msgs::msg::Image &image_pub,
      //const sensor_msgs::msg::CameraInfo &camera_info_pub,
      const std::string &frame_id);
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    void cameraInit();
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
  };
} // namespace vision_tools