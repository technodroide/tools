#include "apriltag_detector/detector.hpp"

#include "rclcpp_components/register_node_macro.hpp"


namespace vision_tools
{
  AprilTagDetector::AprilTagDetector(const rclcpp::NodeOptions &options)
      : Node("apriltag_detector", options)
  {
    using namespace std::chrono_literals;

    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);

    this->declare_parameter("max_hamming_distance", 1);

    /*
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        std::bind(&AprilTagDetector::cameraInfoCallback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&AprilTagDetector::imageCallback, this, std::placeholders::_1));
    */
    tag_publisher_ =
        this->create_publisher<extender_msgs::msg::SharedControlGoalArray>("/tag_detections", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    // publish USB_CAM image_raw and camera_info
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 1);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 1);

    this->get_parameter("max_hamming_distance", max_hamming_distance_);

    std::string prefix = "tag_sizes";
    auto overrides = this->get_node_parameters_interface()->get_parameter_overrides();

    for (auto const &[name, rcl_value] : overrides)
    {
      if (name.compare(0, prefix.size(), prefix) == 0)
      {
        // Extract the ID (the part after "tag_sizes.")
        std::string id_str = name.substr(prefix.size());

        try
        {
          if (!this->has_parameter(name))
          {
            this->declare_parameter(name, rcl_value.get_type());
          }

          double size_value = this->get_parameter(name).as_double();
          int tag_id = std::stoi(id_str.substr(1)); // Skip the dot

          tag_sizes_[tag_id] = size_value;
          RCLCPP_INFO(this->get_logger(), "Successfully loaded Tag ID: %d, Size: %f meters", tag_id,
                      size_value);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse tag parameter '%s': %s", name.c_str(),
                        e.what());
        }
      }
    }

    // Check if we actually loaded anything
    if (tag_sizes_.empty())
    {
      RCLCPP_WARN(this->get_logger(),
                  "No tag_sizes were loaded from YAML. Check the node name and YAML indentation.");
    }
    
    //cv::VideoCapture cap_(0, cv::CAP_V4L2);//("/dev/video1");
    //std::cout<<"INIT ipOpened = "<< cap_.isOpened() << std::endl;


    if (!cap_.open(0, cv::CAP_V4L2))    // <--- camera ID ("/dev/video1");
    {
      RCLCPP_WARN(rclcpp::get_logger("AprilTagDetector::AprilTagDetector"), "Impossible d'ouvrir la caméra.");
    }
    std::cout << "Backend : " << cap_.getBackendName() << std::endl;
    std::cout << "Ouverte : " << cap_.isOpened() << std::endl;
    cameraInit();
    timer_ = this->create_wall_timer(33ms, std::bind(&AprilTagDetector::timer_callback, this));      // 1/30Hz = 0.033s => 33ms
  }

  AprilTagDetector::~AprilTagDetector()
  {
    std::cout << "~AprilTagDetector()" << std::endl;
    std::cout << "Avant release : " << cap_.isOpened() << std::endl;
    if (cap_.isOpened()){
        cap_.release();
      }
    std::cout << "Après release : " << cap_.isOpened() << std::endl;
    apriltag_detector_destroy(td_);
    tag36h11_destroy(tf_);
  }

  void AprilTagDetector::cameraInfoCallback(const sensor_msgs::msg::CameraInfo msg)
  {
    // Extract intrinsics from the K matrix (3x3 row-major)
    // K = [fx 0 cx; 0 fy cy; 0 0 1]
    fx_ = msg.k[0];
    cx_ = msg.k[2];
    fy_ = msg.k[4];
    cy_ = msg.k[5];

    has_camera_info_ = true;

    // RCLCPP_INFO(this->get_logger(), "Camera info received. Intrinsics: fx=%f, fy=%f, cx=%f, cy=%f", fx_, fy_, cx_, cy_);

    info.fx = fx_;
    info.fy = fy_;
    info.cx = cx_;
    info.cy = cy_;

    //camera_info_sub_.reset();
  }

  void AprilTagDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    current_frame_ = cv_bridge::toCvShare(msg, "mono8")->image;
    image_u8_t current_frame_apriltag_ = {
        current_frame_.cols,
        current_frame_.rows,
        current_frame_.cols,
        current_frame_.data,
    };
    
    detections_ = apriltag_detector_detect(td_, &current_frame_apriltag_);
    
    extender_msgs::msg::SharedControlGoalArray detection_array_msg;
    detection_array_msg.header = msg->header;
    
    double err;
    for (int i = 0; i < zarray_size(detections_); i++)
    {
      zarray_get(detections_, i, &det);
      if (det->hamming > max_hamming_distance_)
        continue;
      
      info.det = det;
      info.tagsize = tag_sizes_[det->id];

      std::cout << "id = " << det->id
          << " tag_size = " << tag_sizes_[det->id]
          << std::endl;
      
      apriltag_pose_t pose;
      err = estimate_tag_pose(&info, &pose);

      Eigen::Map<Eigen::Vector3d> translation(pose.t->data);
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(pose.R->data);
      Eigen::Quaterniond q(rotation);
      q.normalize();

      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = msg->header.stamp;
      t.header.frame_id = msg->header.frame_id; 

      t.child_frame_id = "tag_" + std::to_string(det->id);

      t.transform.translation.x = translation.x();
      t.transform.translation.y = translation.y();
      t.transform.translation.z = translation.z();

      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(t);

      extender_msgs::msg::SharedControlGoal detection_msg;
      detection_msg.id = det->id;
      detection_msg.goal_pose.position.x = translation.x();
      detection_msg.goal_pose.position.y = translation.y();
      detection_msg.goal_pose.position.z = translation.z();

      detection_msg.goal_pose.orientation.x = q.x();
      detection_msg.goal_pose.orientation.y = q.y();
      detection_msg.goal_pose.orientation.z = q.z();
      detection_msg.goal_pose.orientation.w = q.w();

      detection_array_msg.goal_array.push_back(detection_msg);
      matd_destroy(pose.R);
      matd_destroy(pose.t);
    }
    
    apriltag_detections_destroy(detections_);
    tag_publisher_->publish(detection_array_msg);
  }

  void AprilTagDetector::cameraInit(){
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);
  }

  void AprilTagDetector::imageDirectCallback(
    cv::VideoCapture &cap,                                                              // lit une image depuis un cv::VideoCapture
    //const sensor_msgs::msg::Image &image_pub,
    //const sensor_msgs::msg::CameraInfo &camera_info_pub,
    const std::string &frame_id)
  {
    cv::Mat frame;
    
    if (!cap.isOpened())
    {
      RCLCPP_WARN(rclcpp::get_logger("AprilTagDetector::imageDirectCallback"), "Impossible d'ouvrir la caméra.");
      return;
    }
    

    if (!cap.read(frame))
    {
      RCLCPP_WARN(rclcpp::get_logger("AprilTagDetector::imageDirectCallback"), "Impossible de lire une image.");
      return;
    }

    std::cout << frame.cols << " x " << frame.rows << std::endl;

    if (frame.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("AprilTagDetector::imageDirectCallback"), "Image vide.");
        return;
    }

    // Timestamp commun aux deux messages
    rclcpp::Time stamp = rclcpp::Clock().now();

    //----------------------------------------
    // image_raw
    //----------------------------------------
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id;

    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(header,"bgr8", frame).toImageMsg();  // ".toImageMsg()" = Convert this message to a ROS sensor_msgs::Image message. 
                                                                                                 // /!\ "cv_bridge::CvImage::toImageMsg()" ne retourne pas un objet "sensor_msgs::msg::Image" mais un pointeur "sensor_msgs::msg::Image::SharedPtr" d'où l'ajout de l' "*" avant "cv_bridge::CvImage"
    //----------------------------------------
    // camera_info
    //----------------------------------------
    sensor_msgs::msg::CameraInfo camera_info;                                       // la convertit en sensor_msgs::msg::Image

    camera_info.header = header;

    camera_info.width = frame.cols;
    camera_info.height = frame.rows;

    camera_info.distortion_model = "plumb_bob";

    // Pas de calibration
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    double fx = 600.0;
    double fy = 600.0;
    double cx = frame.cols / 2.0;
    double cy = frame.rows / 2.0;

    camera_info.k = {
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0
    };

    camera_info.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    camera_info.p = {
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    //----------------------------------------
    // Publication
    //----------------------------------------
    //image_pub_->publish(*image_msg);
    imageCallback(image_msg);
    
    //camera_info_pub_->publish(camera_info);
    cameraInfoCallback(camera_info);
    //cap_.release();
  }

  void AprilTagDetector::timer_callback(){
    imageDirectCallback(cap_,
                        //image_pub_,
                        //camera_info_pub_,
                        "camera_link");
    //std::cout << "YOUR INFO : " << CV_VERSION << std::endl;
  }
} // namespace vision_tools

RCLCPP_COMPONENTS_REGISTER_NODE(vision_tools::AprilTagDetector)

/*
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagDetector>());
  rclcpp::shutdown();
  return 0;
}
*/