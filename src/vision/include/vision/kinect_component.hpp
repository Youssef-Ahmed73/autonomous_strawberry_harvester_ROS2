#ifndef VISION__KINECT_COMPONENT_HPP_
#define VISION__KINECT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <libfreenect.h>
#include "vision/visibility_control.h"

namespace vision
{
class KinectComponent : public rclcpp::Node
{
public:
  VISION_PUBLIC
  explicit KinectComponent(const rclcpp::NodeOptions & options);
  ~KinectComponent() override;

private:
  void loop();

  // Static callbacks required by libfreenect C-API
  static void depth_cb_wrapper(freenect_device *dev, void *depth, uint32_t ts);
  static void video_cb_wrapper(freenect_device *dev, void *video, uint32_t ts);

  // Instance methods that actually process the data
  void process_depth(void *depth);
  void process_video(void *video);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  freenect_context *ctx_;
  freenect_device *dev_;

  cv::Mat depthMat_;
  cv::Mat rgbMat_;
  std::string camera_name_;
};
}  // namespace vision

#endif  // VISION__KINECT_COMPONENT_HPP_