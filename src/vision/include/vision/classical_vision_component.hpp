#ifndef VISION__CLASSICAL_VISION_COMPONENT_HPP_
#define VISION__CLASSICAL_VISION_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "vision/visibility_control.h"

#define FRUIT_SAT_MIN   40
#define FRUIT_VAL_MIN   40
#define FRUIT_VAL_MAX   255

#define RIPE_SAT_MIN    90
#define RIPE_VAL_MIN    40
#define RIPE_VAL_MAX    255

#define MIN_FRUIT_AREA  3000.0
#define ASPECT_MAX      3.5
#define RIPE_THRESHOLD  50.0
#define EMA_ALPHA       0.20

#define PIXEL_GRID_ROWS 20
#define PIXEL_GRID_COLS 20
#define PIXEL_MAP_SIZE  120

namespace vision
{
class ClassicalVisionComponent : public rclcpp::Node
{
public:
  VISION_PUBLIC
  explicit ClassicalVisionComponent(const rclcpp::NodeOptions & options);

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  
  cv::Mat detect_fruit_mask(const cv::Mat& hsv);
  cv::Mat detect_ripe_mask(const cv::Mat& hsv);
  void ripeness_score(const std::vector<cv::Point>& contour, const cv::Mat& fruit_mask, const cv::Mat& ripe_mask, double& ripe_pct, double& unripe_pct);
  void draw_pixel_map(cv::Mat& frame, const std::vector<cv::Point>& contour, const cv::Mat& frame_orig);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

  cv::Mat k_close_;
  cv::Mat k_open_;
  
  double smooth_ripe_;
  double smooth_present_;
};
}  // namespace vision

#endif  // VISION__CLASSICAL_VISION_COMPONENT_HPP_