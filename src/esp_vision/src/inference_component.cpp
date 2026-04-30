#include "esp_vision/inference_component.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>  // Using OpenCV's DNN module for easy pre-processing
#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace esp_vision
{

InferenceComponent::InferenceComponent(const rclcpp::NodeOptions & options)
: Node("inference_component", options)
{
  // Set up ONNX Runtime Session
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "MaskRCNN_Env");
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1); // Set to CPU for now, just like python script

  // Resolve model path from the ROS 2 share directory
  std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("esp_vision");
  std::string model_path = pkg_share_dir + "/models/model.onnx";

  try {
    session_ = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
    RCLCPP_INFO(this->get_logger(), "ONNX Model loaded successfully.");
  } catch (const Ort::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load ONNX model: %s", e.what());
  }

  // Subscribe using unique_ptr for zero-copy IPC
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", 10,
    std::bind(&InferenceComponent::image_callback, this, std::placeholders::_1));
}

void InferenceComponent::image_callback(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!session_) return;

  // Convert ROS Image back to OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image;
  int original_h = frame.rows;
  int original_w = frame.cols;

  // 2. Pre-process (Matching your Python logic)
  // cv::dnn::blobFromImage handles resizing, float32 conversion, and HWC to NCHW format
  cv::Mat blob;
  cv::dnn::blobFromImage(frame, blob, 1.0, cv::Size(640, 640), cv::Scalar(), false, false, CV_32F);

  // Setup ONNX Tensor
  auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  std::vector<int64_t> input_dims = {1, 3, 640, 640};
  
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    memory_info, 
    (float*)blob.data, 
    blob.total(), 
    input_dims.data(), 
    input_dims.size()
  );

  // 3. Run Inference
  std::vector<const char*> input_names = {"images"}; // Adjust to your model's input name
  std::vector<const char*> output_names = {"boxes", "classes", "masks", "scores"}; // Adjust to exporter
  
  auto output_tensors = session_->Run(
    Ort::RunOptions{nullptr}, 
    input_names.data(), &input_tensor, 1, 
    output_names.data(), 4
  );

  // 4 & 5. Parse Outputs and Draw (Abridged for C++)
  // Getting pointers to the output data arrays
  float* boxes_data = output_tensors[0].GetTensorMutableData<float>();
  int64_t* classes_data = output_tensors[1].GetTensorMutableData<int64_t>();
  float* scores_data = output_tensors[3].GetTensorMutableData<float>();

  // Determine how many detections we have based on tensor shape
  auto box_info = output_tensors[0].GetTensorTypeAndShapeInfo();
  size_t num_detections = box_info.GetShape()[0];

  std::vector<std::string> class_names = {"Half-ripe", "Ripe", "Unripe"};

  for (size_t i = 0; i < num_detections; ++i) {
    if (scores_data[i] > 0.5) {
      // Calculate index offset since boxes are flat arrays [N, 4]
      int box_idx = i * 4;
      
      int x1 = static_cast<int>(boxes_data[box_idx] * original_w / 640);
      int y1 = static_cast<int>(boxes_data[box_idx + 1] * original_h / 640);
      int x2 = static_cast<int>(boxes_data[box_idx + 2] * original_w / 640);
      int y2 = static_cast<int>(boxes_data[box_idx + 3] * original_h / 640);
      
      int cls_id = static_cast<int>(classes_data[i]);
      
      // Draw Results
      cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
      std::string label = class_names[cls_id] + ": " + std::to_string(scores_data[i]).substr(0, 4);
      cv::putText(frame, label, cv::Point(x1, y1 - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    }
  }

  // Display for debugging (Optional, usually you publish results instead in ROS)
  cv::imshow("Strawberry Detection C++", frame);
  cv::waitKey(1);
}

}  // namespace esp_vision

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(esp_vision::InferenceComponent)