#include "esp_vision/kinect_component.hpp"
#include <cv_bridge/cv_bridge.h>

namespace esp_vision
{

KinectComponent::KinectComponent(const rclcpp::NodeOptions & options)
: Node("kinect_component", options),
  depthMat_(480, 640, CV_16UC1),
  rgbMat_(480, 640, CV_8UC3)
{
  this->declare_parameter<std::string>("camera_name", "kinect");
  camera_name_ = this->get_parameter("camera_name").as_string();

  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/rgb/image_raw", 10);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/depth/image_raw", 10);

  // Initialize Kinect Context
  if (freenect_init(&ctx_, NULL) < 0) {
    RCLCPP_ERROR(this->get_logger(), "freenect_init() failed");
    return;
  }
  freenect_select_subdevices(ctx_, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));

  if (freenect_open_device(ctx_, &dev_, 0) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not open Kinect device. Is it plugged in?");
    return;
  }

  // Bind this specific C++ class instance to the device driver
  freenect_set_user(dev_, this);
  
  // Set the static C-style wrapper callbacks
  freenect_set_depth_callback(dev_, depth_cb_wrapper);
  freenect_set_video_callback(dev_, video_cb_wrapper);

  freenect_start_depth(dev_);
  freenect_start_video(dev_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(30),
    std::bind(&KinectComponent::loop, this));
}

KinectComponent::~KinectComponent()
{
  if (dev_) {
    freenect_stop_depth(dev_);
    freenect_stop_video(dev_);
    freenect_close_device(dev_);
  }
  if (ctx_) {
    freenect_shutdown(ctx_);
  }
}

// --- Static Wrappers ---
void KinectComponent::depth_cb_wrapper(freenect_device *dev, void *depth, uint32_t /*ts*/)
{
  // Retrieve the C++ class instance from the driver and call the instance method
  KinectComponent* instance = static_cast<KinectComponent*>(freenect_get_user(dev));
  if (instance) instance->process_depth(depth);
}

void KinectComponent::video_cb_wrapper(freenect_device *dev, void *video, uint32_t /*ts*/)
{
  KinectComponent* instance = static_cast<KinectComponent*>(freenect_get_user(dev));
  if (instance) instance->process_video(video);
}

// --- Instance Processing ---
void KinectComponent::process_depth(void *depth)
{
  memcpy(depthMat_.data, depth, 640 * 480 * 2);
}

void KinectComponent::process_video(void *video)
{
  memcpy(rgbMat_.data, video, 640 * 480 * 3);
}

// --- Main Loop ---
void KinectComponent::loop()
{
  if (ctx_) {
    freenect_process_events(ctx_);
  }

  auto now = this->now();

  cv::Mat bgr;
  cv::cvtColor(rgbMat_, bgr, cv::COLOR_RGB2BGR);

  std_msgs::msg::Header header;
  header.stamp = now;
  header.frame_id = camera_name_ + "_link";

  // Pre-allocate Unique Pointers for ZERO-COPY IPC
  auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();

  cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg(*rgb_msg);
  cv_bridge::CvImage(header, "mono16", depthMat_).toImageMsg(*depth_msg);

  rgb_pub_->publish(std::move(rgb_msg));
  depth_pub_->publish(std::move(depth_msg));
}

}  // namespace esp_vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(esp_vision::KinectComponent)