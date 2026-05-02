cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# --- Build Camera Component ---
add_library(camera_component SHARED src/camera_component.cpp)
target_include_directories(camera_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_component ${OpenCV_LIBS})
ament_target_dependencies(camera_component rclcpp rclcpp_components sensor_msgs cv_bridge)
rclcpp_components_register_node(camera_component PLUGIN "vision::CameraComponent" EXECUTABLE camera_node)


# --- Find ONNX Runtime ---
set(ONNXRUNTIME_DIR "/opt/onnxruntime/onnxruntime-linux-x64-gpu-1.17.1") 

find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_DIR}/lib)
if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "ONNX Runtime library not found!")
endif()

# --- Build Inference Component ---
add_library(inference_component SHARED src/inference_component.cpp)
target_include_directories(inference_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_DIR}/include # Crucial for finding cuda_provider_factory.h
)
target_link_libraries(inference_component ${OpenCV_LIBS} ${ONNXRUNTIME_LIB})
ament_target_dependencies(inference_component rclcpp rclcpp_components sensor_msgs cv_bridge)
rclcpp_components_register_node(inference_component PLUGIN "vision::InferenceComponent" EXECUTABLE inference_node)


# --- Build Kinect Component ---
add_library(kinect_component SHARED src/kinect_component.cpp)
target_include_directories(kinect_component PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Notice we add "freenect" to the target_link_libraries here
target_link_libraries(kinect_component ${OpenCV_LIBS} freenect)
ament_target_dependencies(kinect_component rclcpp rclcpp_components sensor_msgs cv_bridge)
rclcpp_components_register_node(kinect_component PLUGIN "vision::KinectComponent" EXECUTABLE kinect_node)

# Notice inference_component is removed from the TARGETS list below
install(TARGETS 
  camera_component 
  kinect_component
  inference_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()