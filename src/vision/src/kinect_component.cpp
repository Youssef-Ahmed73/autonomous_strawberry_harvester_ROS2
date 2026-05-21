#include "vision/kinect_component.hpp"
#include <cv_bridge/cv_bridge.h>

namespace vision
{

KinectComponent::KinectComponent(const rclcpp::NodeOptions & options)
: Node("kinect_component", options),
  depthMat_(480, 640, CV_16UC1),
  rgbMat_(480, 640, CV_8UC3)
{
  this->declare_parameter<std::string>("camera_name", "kinect");
  camera_name_ = this->get_parameter("camera_name").as_string();

  // Initialize Publishers
  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/rgb/image_raw", 10);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("kinect/depth/image_raw", 10);
  depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("kinect/depth/camera_info", 10);

  // Setup standard Kinect v1 Intrinsics
  camera_info_msg_.header.frame_id = camera_name_ + "_link";
  camera_info_msg_.height = 480;
  camera_info_msg_.width = 640;
  camera_info_msg_.distortion_model = "plumb_bob";
  camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  camera_info_msg_.k = {594.21, 0.0, 339.5, 0.0, 591.04, 242.7, 0.0, 0.0, 1.0};
  camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info_msg_.p = {594.21, 0.0, 339.5, 0.0, 0.0, 591.04, 242.7, 0.0, 0.0, 0.0, 1.0, 0.0};

  if (freenect_init(&ctx_, NULL) < 0) {
    RCLCPP_ERROR(this->get_logger(), "freenect_init() failed");
    return;
  }
  freenect_select_subdevices(ctx_, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));

  if (freenect_open_device(ctx_, &dev_, 0) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not open Kinect device. Is it plugged in?");
    return;
  }

  freenect_set_user(dev_, this);
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
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(depthMat_.data, depth, 640 * 480 * 2);
}

void KinectComponent::process_video(void *video)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  memcpy(rgbMat_.data, video, 640 * 480 * 3);
}

// --- Main Loop ---
void KinectComponent::loop()
{
  if (ctx_) {
    freenect_process_events(ctx_);
  }

  auto now = this->now();
  cv::Mat bgr, depth_copy;

  // Safely copy the data out of the variables the USB thread is writing to
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cv::cvtColor(rgbMat_, bgr, cv::COLOR_RGB2BGR);
    depthMat_.copyTo(depth_copy);
  }

  std_msgs::msg::Header header;
  header.stamp = now;
  header.frame_id = camera_name_ + "_link";

  // Pre-allocate Unique Pointers for ZERO-COPY IPC
  auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
  
  // Create unique pointer for Camera Info and sync timestamp
  camera_info_msg_.header.stamp = now;
  auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_msg_);

  cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg(*rgb_msg);
  cv_bridge::CvImage(header, "mono16", depth_copy).toImageMsg(*depth_msg);

  rgb_pub_->publish(std::move(rgb_msg));
  depth_pub_->publish(std::move(depth_msg));
  depth_info_pub_->publish(std::move(info_msg));
}

}  // namespace vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision::KinectComponent)