#ifndef ESP_VISION__INFERENCE_COMPONENT_HPP_
#define ESP_VISION__INFERENCE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <onnxruntime_cxx_api.h>
#include "esp_vision/visibility_control.h"

namespace esp_vision
{
class InferenceComponent : public rclcpp::Node
{
public:
  ESP_VISION_PUBLIC
  explicit InferenceComponent(const rclcpp::NodeOptions & options);

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::unique_ptr<Ort::Session> session_;
};
}  // namespace esp_vision

#endif  // ESP_VISION__INFERENCE_COMPONENT_HPP_