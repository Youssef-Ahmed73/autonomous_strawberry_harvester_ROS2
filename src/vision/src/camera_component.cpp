#include "vision/camera_component.hpp"
#include <cv_bridge/cv_bridge.h>

namespace vision
{

CameraComponent::CameraComponent(const rclcpp::NodeOptions & options)
: Node("camera_component", options)
{
  this->declare_parameter<std::string>("stream_url", "http://192.168.100.130:8080/video");
  this->declare_parameter<std::string>("camera_name", "cam_1");

  stream_url_ = this->get_parameter("stream_url").as_string();
  camera_name_ = this->get_parameter("camera_name").as_string();

  cap_.open(stream_url_);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open video stream: %s", stream_url_.c_str());
  }

  //image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&CameraComponent::timer_callback, this));
}

CameraComponent::~CameraComponent()
{
  cap_.release();
}

void CameraComponent::timer_callback()
{
  if (!cap_.isOpened()) return;

  cv::Mat frame;
  cap_ >> frame;

  if (frame.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty frame received from IP stream");
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = camera_name_;

  // Pre-allocate the memory for the UniquePtr to enable zero-copy
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*msg);

  image_pub_->publish(std::move(msg));
}

}  // namespace vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision::CameraComponent)