#include "vision/classical_vision_component.hpp"

namespace vision
{

ClassicalVisionComponent::ClassicalVisionComponent(const rclcpp::NodeOptions & options)
: Node("classical_vision_component", options),
  smooth_ripe_(0.0),
  smooth_present_(0.0)
{
  this->declare_parameter("debug_viz", true);

  k_close_ = cv::Mat::ones(15, 15, CV_8U);
  k_open_ = cv::Mat::ones(5, 5, CV_8U);

  rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
  
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", qos_profile,
    std::bind(&ClassicalVisionComponent::image_callback, this, std::placeholders::_1),
    sub_opt);

  detections_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detections_debug", 10);

  ripeness_srv_ = this->create_service<ashr_interfaces::srv::CheckRipeness>(
    "verify_ripeness",
    std::bind(&ClassicalVisionComponent::check_ripeness_callback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_
  );

  RCLCPP_INFO(this->get_logger(), "Classical Vision Component initialized.");
}

cv::Mat ClassicalVisionComponent::detect_fruit_mask(const cv::Mat& hsv)
{
  cv::Mat fruit_mask, background, final_mask;
  cv::inRange(hsv, cv::Scalar(0, FRUIT_SAT_MIN, FRUIT_VAL_MIN), cv::Scalar(180, 255, FRUIT_VAL_MAX), fruit_mask);
  cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 35, 255), background);
  
  cv::bitwise_not(background, background);
  cv::bitwise_and(fruit_mask, background, final_mask);

  cv::morphologyEx(final_mask, final_mask, cv::MORPH_CLOSE, k_close_);
  cv::morphologyEx(final_mask, final_mask, cv::MORPH_OPEN, k_open_);
  
  return final_mask;
}

cv::Mat ClassicalVisionComponent::detect_ripe_mask(const cv::Mat& hsv)
{
  cv::Mat r1, r2, ripe_mask;
  cv::inRange(hsv, cv::Scalar(0, RIPE_SAT_MIN, RIPE_VAL_MIN), cv::Scalar(12, 255, RIPE_VAL_MAX), r1);
  cv::inRange(hsv, cv::Scalar(163, RIPE_SAT_MIN, RIPE_VAL_MIN), cv::Scalar(180, 255, RIPE_VAL_MAX), r2);
  cv::add(r1, r2, ripe_mask);
  return ripe_mask;
}

void ClassicalVisionComponent::ripeness_score(const std::vector<cv::Point>& contour, const cv::Mat& fruit_mask, const cv::Mat& ripe_mask, double& ripe_pct, double& unripe_pct)
{
  cv::Mat shape = cv::Mat::zeros(fruit_mask.size(), CV_8U);
  std::vector<std::vector<cv::Point>> contours = {contour};
  cv::drawContours(shape, contours, -1, cv::Scalar(255), cv::FILLED);

  cv::Mat masked_fruit, masked_ripe;
  cv::bitwise_and(fruit_mask, shape, masked_fruit);
  cv::bitwise_and(ripe_mask, shape, masked_ripe);

  int total_px = cv::countNonZero(masked_fruit);
  if (total_px == 0) {
    ripe_pct = 0.0;
    unripe_pct = 0.0;
    return;
  }

  int ripe_px = cv::countNonZero(masked_ripe);
  int unripe_approx = total_px - ripe_px;

  ripe_pct = (static_cast<double>(ripe_px) / total_px) * 100.0;
  unripe_pct = (static_cast<double>(unripe_approx) / total_px) * 100.0;
}

void ClassicalVisionComponent::draw_pixel_map(cv::Mat& frame, const std::vector<cv::Point>& contour, const cv::Mat& frame_orig)
{
  cv::Rect bbox = cv::boundingRect(contour);
  cv::Mat shape_mask = cv::Mat::zeros(frame_orig.size(), CV_8U);
  std::vector<std::vector<cv::Point>> contours = {contour};
  cv::drawContours(shape_mask, contours, -1, cv::Scalar(255), cv::FILLED);

  int cell_w = std::max(bbox.width / PIXEL_GRID_COLS, 1);
  int cell_h = std::max(bbox.height / PIXEL_GRID_ROWS, 1);
  int cell_draw = PIXEL_MAP_SIZE / PIXEL_GRID_COLS;
  int map_x = 10;
  int map_y = frame.rows - PIXEL_MAP_SIZE - 10;

  for (int row = 0; row < PIXEL_GRID_ROWS; ++row) {
    for (int col = 0; col < PIXEL_GRID_COLS; ++col) {
      int cx = bbox.x + col * cell_w;
      int cy = bbox.y + row * cell_h;
      int cx2 = std::min(cx + cell_w, frame_orig.cols);
      int cy2 = std::min(cy + cell_h, frame_orig.rows);

      if (cx2 <= cx || cy2 <= cy) continue;

      cv::Mat cell_mask = shape_mask(cv::Rect(cx, cy, cx2 - cx, cy2 - cy));
      if (cv::countNonZero(cell_mask) == 0) continue;

      cv::Mat cell_pixels;
      frame_orig(cv::Rect(cx, cy, cx2 - cx, cy2 - cy)).copyTo(cell_pixels, cell_mask);
      cv::Scalar avg = cv::mean(cell_pixels, cell_mask);

      int dx = map_x + col * cell_draw;
      int dy = map_y + row * cell_draw;
      cv::rectangle(frame, cv::Point(dx, dy), cv::Point(dx + cell_draw, dy + cell_draw), avg, -1);
    }
  }

  cv::rectangle(frame, cv::Point(map_x - 2, map_y - 2), cv::Point(map_x + PIXEL_MAP_SIZE + 2, map_y + PIXEL_MAP_SIZE + 2), cv::Scalar(200, 200, 200), 1);
  cv::putText(frame, "pixel map", cv::Point(map_x, map_y - 6), cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(180, 180, 180), 1);
}

void ClassicalVisionComponent::check_ripeness_callback(
  const std::shared_ptr<ashr_interfaces::srv::CheckRipeness::Request> request,
  std::shared_ptr<ashr_interfaces::srv::CheckRipeness::Response> response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Ripeness check requested. Averaging %d frames...", target_frames_);

  std::unique_lock<std::mutex> lock(eval_mutex_);
  
  frames_collected_ = 0;
  accum_ripe_ = 0.0;
  evaluation_requested_ = true;

  bool success = eval_cv_.wait_for(lock, std::chrono::seconds(2), [this]{ return frames_collected_ >= target_frames_; });

  evaluation_requested_ = false;

  if (success) {
    double average_ripe_pct = accum_ripe_ / target_frames_;
    response->ripeness_percentage = average_ripe_pct;
    response->is_ripe = (average_ripe_pct > RIPE_THRESHOLD);
    
    RCLCPP_INFO(this->get_logger(), "Inspection complete. Ripe Pct: %.2f%%. Is Ripe: %s", 
                average_ripe_pct, response->is_ripe ? "TRUE" : "FALSE");
  } else {
    RCLCPP_WARN(this->get_logger(), "Ripeness check timed out! Camera might be disconnected.");
    response->ripeness_percentage = 0.0;
    response->is_ripe = false;
  }
}

void ClassicalVisionComponent::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  bool debug_viz = this->get_parameter("debug_viz").as_bool();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  cv::Mat frame_orig;
  if (debug_viz) {
    frame_orig = frame.clone();
  }

  double frame_area = frame.rows * frame.cols;
  cv::Mat fruit_mask = detect_fruit_mask(frame);
  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  
  fruit_mask = detect_fruit_mask(hsv);
  cv::Mat ripe_mask = detect_ripe_mask(hsv);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(fruit_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double raw_present = 0.0, raw_ripe = 0.0;
  vision_msgs::msg::Detection2DArray detections_msg;
  detections_msg.header = msg->header;

  double current_frame_ripe_pct = 0.0;

  if (!contours.empty()) {
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
      return cv::contourArea(a) > cv::contourArea(b);
    });

    for (const auto& cnt : contours) {
      double area = cv::contourArea(cnt);
      if (area < MIN_FRUIT_AREA) break;

      std::vector<cv::Point> hull;
      cv::convexHull(cnt, hull);
      double hull_area = cv::contourArea(hull);
      cv::Rect bbox = cv::boundingRect(hull);

      double aspect_ratio = static_cast<double>(std::max(bbox.width, bbox.height)) / std::max(std::min(bbox.width, bbox.height), 1);
      if (aspect_ratio > ASPECT_MAX) continue;

      double ripe_pct, unripe_pct;
      ripeness_score(cnt, fruit_mask, ripe_mask, ripe_pct, unripe_pct);
      
      raw_present = std::min((hull_area / frame_area) * 250.0, 100.0);
      raw_ripe = ripe_pct;
      bool is_ripe = ripe_pct > RIPE_THRESHOLD;
      
      current_frame_ripe_pct = ripe_pct;

      if (debug_viz) {
        cv::Scalar clr_cnt = is_ripe ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 140, 255);
        cv::Scalar clr_hull = is_ripe ? cv::Scalar(0, 255, 180) : cv::Scalar(0, 200, 255);

        std::vector<std::vector<cv::Point>> hulls = {hull};
        std::vector<std::vector<cv::Point>> cnts = {cnt};
        cv::drawContours(frame, hulls, -1, clr_hull, 1);
        cv::drawContours(frame, cnts, -1, clr_cnt, 2);

        int bar_x = bbox.x + 4;
        int bar_y = std::min(bbox.y + bbox.height + 8, frame.rows - 20);
        int bar_w = std::max(bbox.width - 8, 10);
        cv::rectangle(frame, cv::Point(bar_x, bar_y), cv::Point(bar_x + bar_w, bar_y + 10), cv::Scalar(50, 50, 50), -1);
        cv::rectangle(frame, cv::Point(bar_x, bar_y), cv::Point(bar_x + static_cast<int>(bar_w * ripe_pct / 100.0), bar_y + 10), is_ripe ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 120, 255), -1);

        char text[100];
        snprintf(text, sizeof(text), "ripe:%.0f%% unripe:%.0f%%", ripe_pct, unripe_pct);
        cv::putText(frame, text, cv::Point(bbox.x, std::max(bbox.y - 8, 20)), cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(0, 255, 220), 1);

        draw_pixel_map(frame, cnt, frame_orig);
      }

      vision_msgs::msg::Detection2D detection;
      detection.bbox.center.position.x = bbox.x + bbox.width / 2.0;
      detection.bbox.center.position.y = bbox.y + bbox.height / 2.0;
      detection.bbox.size_x = bbox.width;
      detection.bbox.size_y = bbox.height;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = is_ripe ? "ripe_strawberry" : "unripe_strawberry";
      hyp.hypothesis.score = ripe_pct / 100.0;
      detection.results.push_back(hyp);
      
      detections_msg.detections.push_back(detection);
      break; 
    }
  }

  if (evaluation_requested_.load()) {
    std::lock_guard<std::mutex> lock(eval_mutex_);
    if (frames_collected_ < target_frames_) {
      accum_ripe_ += current_frame_ripe_pct;
      frames_collected_++;
      if (frames_collected_ >= target_frames_) {
        eval_cv_.notify_one();
      }
    }
  }

  double decay = (raw_present == 0.0) ? 0.70 : 1.0;
  smooth_present_ = (EMA_ALPHA * raw_present + (1.0 - EMA_ALPHA) * smooth_present_) * decay;
  smooth_ripe_ = (EMA_ALPHA * raw_ripe + (1.0 - EMA_ALPHA) * smooth_ripe_) * decay;

  detections_pub_->publish(detections_msg);

  if (debug_viz) {
    sensor_msgs::msg::Image::SharedPtr debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    debug_pub_->publish(*debug_msg);
  }
}

}  // namespace vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision::ClassicalVisionComponent)