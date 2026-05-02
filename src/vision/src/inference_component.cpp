#include "vision/inference_component.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp> // Needed for cv::resize and cv::addWeighted
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm> 
#include <exception>

namespace vision
{

static std::unique_ptr<Ort::Env> g_ort_env = nullptr;

InferenceComponent::InferenceComponent(const rclcpp::NodeOptions & options)
: Node("inference_component", options)
{
  // Declare the debug parameter (Default to true so you can see it right away)
  this->declare_parameter("debug_viz", true);

  if (!g_ort_env) {
    g_ort_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "MaskRCNN_Env");
  }

  Ort::SessionOptions session_options;
  
  // --- FIX 1: Threads and Graph Optimization ---
  session_options.SetIntraOpNumThreads(4); // Give CPU fallback layers room to breathe
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC); // Stop ORT from breaking the model graph
  
  // Enable CUDA Execution Provider 
  OrtCUDAProviderOptions cuda_options;
  cuda_options.device_id = 0; // Targets RTX 3050

  // Optional but recommended for memory efficiency:
  cuda_options.arena_extend_strategy = 0; 
  // cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive; <-- REMOVED to fix the 60-second freeze
  cuda_options.do_copy_in_default_stream = 1;

  try {
    session_options.AppendExecutionProvider_CUDA(cuda_options);
    RCLCPP_INFO(this->get_logger(), "CUDA Execution Provider appended successfully.");
  } catch (const Ort::Exception& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to append CUDA EP, falling back to CPU: %s", e.what());
  }

  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("vision");
  std::string model_path = pkg_share_dir + "/models/model.onnx";

  try {
    session_ = std::make_unique<Ort::Session>(*g_ort_env, model_path.c_str(), session_options);
    RCLCPP_INFO(this->get_logger(), "ONNX Model loaded successfully.");
  } catch (const Ort::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
  }

  rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", qos_profile, std::bind(&InferenceComponent::image_callback, this, std::placeholders::_1));

  // We will publish the debug image here
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("detections_debug", 10);
}

void InferenceComponent::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!session_) return;

  // Check if we should spend CPU cycles drawing
  bool debug_viz = this->get_parameter("debug_viz").as_bool();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    return;
  }

  if (cv_ptr->image.empty()) return;
  cv::Mat frame = cv_ptr->image;

  try {
    cv::Mat blob;
    // Keeping blobFromImage barebones to perfectly match your Python astype(np.float32) logic
    cv::dnn::blobFromImage(frame, blob, 1.0, cv::Size(640, 640), cv::Scalar(), false, false, CV_32F);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<int64_t> input_dims = {3, 640, 640}; 
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      memory_info, (float*)blob.data, blob.total(), input_dims.data(), input_dims.size()
    );

    std::vector<const char*> input_names = {"x.1"}; 
    
    // We brought the mask tensor ("value") back at index 3
    std::vector<const char*> output_names = {"boxes.35", "value.3", "value.7", "value"};
    
    Ort::RunOptions run_options; 
    auto output_tensors = session_->Run(
      run_options, input_names.data(), &input_tensor, 1, output_names.data(), 4
    );

    size_t box_elements = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    size_t num_detections = box_elements / 4;

if (num_detections > 0) {
      float* boxes_data = output_tensors[0].GetTensorMutableData<float>();
      int64_t* classes_data = output_tensors[1].GetTensorMutableData<int64_t>();
      
      // --- THE INVISIBLE ENEMY DEFEATED ---
      // We swap these indices to perfectly match the Python layout!
      // Index 2 is the Masks tensor. Index 3 is the Scores tensor.
      float* masks_data = output_tensors[2].GetTensorMutableData<float>();
      float* scores_data = output_tensors[3].GetTensorMutableData<float>();

      // Dynamically get the size of the mask from output_tensors[2] (not 3!)
      auto mask_shape = output_tensors[2].GetTensorTypeAndShapeInfo().GetShape();
      int mask_h = mask_shape[mask_shape.size() - 2];
      int mask_w = mask_shape[mask_shape.size() - 1];
      
      // Calculate total elements per detection using the correct mask tensor
      size_t elements_per_detection = output_tensors[2].GetTensorTypeAndShapeInfo().GetElementCount() / num_detections;

      std::vector<std::string> class_names = {"Half-ripe", "Ripe", "Unripe"};
      int original_w = frame.cols;
      int original_h = frame.rows;

      for (size_t i = 0; i < num_detections; ++i) {
        if (scores_data[i] > 0.85) { // NOW this is actually checking the real confidence score
          int box_idx = i * 4;
          int x1 = static_cast<int>(boxes_data[box_idx] * original_w / 640);
          int y1 = static_cast<int>(boxes_data[box_idx + 1] * original_h / 640);
          int x2 = static_cast<int>(boxes_data[box_idx + 2] * original_w / 640);
          int y2 = static_cast<int>(boxes_data[box_idx + 3] * original_h / 640);
          
          x1 = std::max(0, std::min(x1, original_w - 1));
          y1 = std::max(0, std::min(y1, original_h - 1));
          x2 = std::max(0, std::min(x2, original_w - 1));
          y2 = std::max(0, std::min(y2, original_h - 1));

          if (x2 > x1 && y2 > y1) {
            int cls_id = static_cast<int>(classes_data[i]);
            
            if (debug_viz) {
              std::string label = "Unknown";
              cv::Scalar color = cv::Scalar(255, 0, 255); // Default Purple
              if (cls_id >= 0 && cls_id < (int)class_names.size()) {
                  label = class_names[cls_id];
                  if (label == "Ripe") color = cv::Scalar(0, 0, 255); // Red
                  else if (label == "Half-ripe") color = cv::Scalar(0, 165, 255); // Orange
                  else if (label == "Unripe") color = cv::Scalar(0, 255, 0); // Green
              }

              // 1. Draw Bounding Box & Text
              cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
              label += ": " + std::to_string(scores_data[i]).substr(0, 4);
              cv::putText(frame, label, cv::Point(x1, y1 - 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

              // 2. Process and Draw Segmentation Mask
              int mask_offset = (i * elements_per_detection) + (cls_id * mask_h * mask_w);
              cv::Mat raw_mask(mask_h, mask_w, CV_32F, masks_data + mask_offset);
              
              cv::Mat resized_mask;
              cv::resize(raw_mask, resized_mask, cv::Size(x2 - x1, y2 - y1));

              cv::Mat roi = frame(cv::Rect(x1, y1, x2 - x1, y2 - y1));
              cv::Mat colorMask = cv::Mat::zeros(roi.size(), roi.type());
              colorMask.setTo(color, resized_mask > 0.5); 

              cv::addWeighted(roi, 1.0, colorMask, 0.5, 0.0, roi);
            }
          }
        }
      }
    }
  } catch (const Ort::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "ORT Exception: %s", e.what());
  }

  // Only publish the heavy image message if debug mode is on
  if (debug_viz) {
    image_pub_->publish(*cv_ptr->toImageMsg());
  }
}

}  // namespace vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vision::InferenceComponent)