#ifndef VISION__TARGET_LOCATOR_COMPONENT_HPP_
#define VISION__TARGET_LOCATOR_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace vision
{

class TargetLocatorComponent : public rclcpp::Node
{
public:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;

  explicit TargetLocatorComponent(const rclcpp::NodeOptions & options);

private:
  void sync_callback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
};

}  // namespace vision

#endif  // VISION__TARGET_LOCATOR_COMPONENT_HPP_