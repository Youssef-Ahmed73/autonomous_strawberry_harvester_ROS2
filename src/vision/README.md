# vision

## Purpose

`vision` is the perception package for the Probot system. It captures camera images, performs object detection (via deep learning or classical image processing), and converts detections into 3D target poses.

## System architecture

The package is composed of five core component nodes:

1.  **`CameraComponent`**
    * Captures an IP camera stream using OpenCV.
    * Publishes raw images on `image_raw`.
    * Uses `rclcpp::SensorDataQoS` for low-latency image streaming.

2.  **`InferenceComponent`**
    * Subscribes to `image_raw`.
    * Loads an ONNX deep learning model (e.g., Mask R-CNN) from `share/vision/models/model.onnx`.
    * Publishes detection results on `detections` and optional debug images on `detections_debug`.

3.  **`ClassicalVisionComponent`** *(Interchangeable with InferenceComponent)*
    * Subscribes to `image_raw`.
    * Uses classical OpenCV operations (HSV color thresholding, morphological operations, contour detection) to identify strawberries and evaluate ripeness based on pixel density.
    * Publishes standard bounding boxes on `detections` and optional debug pixel-map grids on `detections_debug`.

4.  **`KinectComponent`**
    * Interfaces with Kinect hardware through `libfreenect`.
    * Publishes `kinect/rgb/image_raw`, `kinect/depth/image_raw`, and `kinect/depth/camera_info`.
    * Uses a fixed camera intrinsics model and sets `frame_id` to `<camera_name>_link`.

5.  **`TargetLocatorComponent`**
    * Subscribes to `detections`, depth image, and camera info.
    * Uses `message_filters::ApproximateTime` to synchronize these 3 streams.
    * Computes a 3D pose for the first valid detection and publishes it on `target_pose`.

## Launching

* `vision/launch/kinect.launch.py` starts the Kinect, Inference, and Target Locator components in a multi-threaded component container (`component_container_mt`).
* `vision/launch/classical_vision.launch.py` launches the camera alongside the `ClassicalVisionComponent` for hardware setups where GPU inference is unnecessary or unavailable.
* The `InferenceComponent` and `ClassicalVisionComponent` are designed as drop-in replacements for one another. They both expect input on `image_raw` and produce the exact same `vision_msgs/msg/Detection2DArray` structure on `detections`.

## Communication graph

* `CameraComponent` → `image_raw` → `InferenceComponent` **OR** `ClassicalVisionComponent`
* `InferenceComponent` **OR** `ClassicalVisionComponent` → `detections` → `TargetLocatorComponent`
* `KinectComponent` → `kinect/depth/image_raw` & `kinect/depth/camera_info` → `TargetLocatorComponent`
* `TargetLocatorComponent` → `target_pose`

## Synchronization strategy

* `TargetLocatorComponent` uses `message_filters::Synchronizer<ApproximateTime>` with a queue size of 10.
* It aligns `detections`, `depth`, and `camera_info` streams approximately rather than strictly.
* This means the package tolerates small timestamp differences between image processing and depth capture, assuming the messages are close enough temporally to represent the same scene.

## Frame conventions

* `CameraComponent` sets image header `frame_id` to the configured `camera_name` parameter (default `cam_1`).
* `KinectComponent` sets `frame_id` to `kinect_link` by default.
* `TargetLocatorComponent` reuses the depth message frame for the published `target_pose.header.frame_id`.
* The computed target pose is expressed in the Kinect camera frame, not in world or robot body coordinates.

## Hidden coupling points

* The Vision pipeline expects the exact topic name `image_raw` unless remapped in the launch files.
* `TargetLocatorComponent` expects exactly `detections`, `kinect/depth/image_raw`, and `kinect/depth/camera_info`.
* `gazebo_bringup.launch.py` remaps those topics to Gazebo's bridged camera topics, creating the sole bridge between simulation and the vision stack.
* The package assumes the ONNX runtime is installed under `/opt/onnxruntime/onnxruntime-linux-x64-gpu-1.17.1`; changing that path requires `CMakeLists.txt` updates.
* `target_pose` is published with a neutral quaternion (`w=1`), so orientation is intentionally fixed.

## Behavioral constraints

* If changing the depth image encoding or camera intrinsics, update the `TargetLocatorComponent` depth conversion logic.
* Keep the hardcoded ONNX provider path in sync with installed runtime versions.
* Do not assume accurate pose output if the depth camera is not synchronized with detection frames; the current pipeline uses approximate time.
* **Performance Parameter:** Both detection components feature a `debug_viz` ROS 2 parameter. Ensure this is set to `false` during physical deployment to prevent the node from wasting CPU cycles drawing visual overlays and wasting bandwidth publishing them.

## Performance assumptions

* `image_raw` is expected to be a high-rate sensor stream, utilizing a sensor QoS profile.
* `InferenceComponent` is designed for GPU acceleration but will fall back to the CPU if CUDA initialization fails. 
* `ClassicalVisionComponent` runs entirely on the CPU using optimized OpenCV binaries, making it highly suitable for resource-constrained edge devices.
* Debug visualizations (bounding boxes, masks, and ripeness progress bars) introduce heavy computational overhead and should only be used for development/troubleshooting.

## Useful details for AI context

* This package is the perception bridge from raw cameras to 3D target poses.
* It is responsible for the exact topic path and timestamp alignment used by higher-level planners.
* Both Deep Learning and Classical machine vision architectures are explicitly supported via standard ROS 2 Component interfaces.