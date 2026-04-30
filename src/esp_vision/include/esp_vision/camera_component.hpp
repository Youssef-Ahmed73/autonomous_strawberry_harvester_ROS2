#ifndef ESP_VISION__CAMERA_COMPONENT_HPP_
#define ESP_VISION__CAMERA_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "esp_vision/visibility_control.h"

namespace esp_vision
{
class CameraComponent : public rclcpp::Node
{
public:
  ESP_VISION_PUBLIC
  explicit CameraComponent(const rclcpp::NodeOptions & options);
  ~CameraComponent() override;

private:
  void timer_callback();
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::string camera_name_;
  std::string stream_url_;
};
}  // namespace esp_vision

#endif  // ESP_VISION__CAMERA_COMPONENT_HPP_