#ifndef VISION__INFERENCE_COMPONENT_HPP_
#define VISION__INFERENCE_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp> // <-- Added for det_pub_
#include <onnxruntime_cxx_api.h>
#include "vision/visibility_control.h"


namespace vision
{
class InferenceComponent : public rclcpp::Node
{
public:
  VISION_PUBLIC
  explicit InferenceComponent(const rclcpp::NodeOptions & options);

private:
  void image_callback(sensor_msgs::msg::Image::UniquePtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; 
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  std::unique_ptr<Ort::Session> session_;
};
}  // namespace vision

#endif  // VISION__INFERENCE_COMPONENT_HPP_