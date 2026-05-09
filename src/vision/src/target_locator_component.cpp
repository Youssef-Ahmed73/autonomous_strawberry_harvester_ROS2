#include "vision/target_locator_component.hpp"
#include <cv_bridge/cv_bridge.h>

namespace vision
{

TargetLocatorComponent::TargetLocatorComponent(const rclcpp::NodeOptions & options)
: Node("target_locator_component", options)
{
  // Initialize Publishers
  target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

  // Initialize the Message Filter Subscribers
  det_sub_.subscribe(this, "detections", rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, "kinect/depth/image_raw", rmw_qos_profile_sensor_data);
  info_sub_.subscribe(this, "kinect/depth/camera_info", rmw_qos_profile_sensor_data);

  // Initialize the Synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), det_sub_, depth_sub_, info_sub_);

  // Register the callback
  sync_->registerCallback(
    std::bind(&TargetLocatorComponent::sync_callback, this, 
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(this->get_logger(), "Target Locator Initialized. Waiting for synchronized topics...");
}

void TargetLocatorComponent::sync_callback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
{
  if (det_msg->detections.empty()) return; // Nothing detected, skip

  // Convert ROS Depth Image to OpenCV Matrix for easy pixel access
  cv_bridge::CvImageConstPtr cv_depth;
  try {
    cv_depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Extract Camera Intrinsics
  double fx = info_msg->k[0];
  double cx = info_msg->k[2];
  double fy = info_msg->k[4];
  double cy = info_msg->k[5];

  // Loop through all detections
  for (const auto& detection : det_msg->detections) {
    
    // Get the 2D center pixel of the bounding box (u, v)
    int u = static_cast<int>(detection.bbox.center.position.x);
    int v = static_cast<int>(detection.bbox.center.position.y);

    // Ensure pixel is within image bounds
    if (u < 0 || u >= cv_depth->image.cols || v < 0 || v >= cv_depth->image.rows) continue;

    double z_meters = 0.0;

    // Check depth encoding (Real Kinect vs Gazebo)
    if (cv_depth->image.type() == CV_16UC1) {
      uint16_t depth_mm = cv_depth->image.at<uint16_t>(v, u);
      if (depth_mm == 0) continue; 
      z_meters = depth_mm / 1000.0;
    } 
    else if (cv_depth->image.type() == CV_32FC1) {
      float depth_m = cv_depth->image.at<float>(v, u);
      if (std::isnan(depth_m) || std::isinf(depth_m)) continue;
      z_meters = depth_m;
    }

    // Pinhole Camera Math to find X and Y in the camera's 3D frame
    double x_meters = (u - cx) * z_meters / fx;
    double y_meters = (v - cy) * z_meters / fy;

    // Publish the 3D target for MoveIt
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.stamp = depth_msg->header.stamp;
    target_pose.header.frame_id = depth_msg->header.frame_id;
    
    target_pose.pose.position.x = x_meters;
    target_pose.pose.position.y = y_meters;
    target_pose.pose.position.z = z_meters;

    // Point the gripper straight at the target (Neutral quaternion)
    target_pose.pose.orientation.w = 1.0; 

    target_pub_->publish(target_pose);

    RCLCPP_INFO(this->get_logger(), "Strawberry located at X:%.3f, Y:%.3f, Z:%.3f", 
                x_meters, y_meters, z_meters);
                
    break; // Break after the first valid detection 
  }
}

}  // namespace vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision::TargetLocatorComponent)