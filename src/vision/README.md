# vision

## Purpose

`vision` is the perception package for the Probot system. It captures camera images, performs object inference, and converts detections into 3D target poses.

## System architecture

The package is composed of four core component nodes:

1. `CameraComponent`
   - Captures an IP camera stream using OpenCV.
   - Publishes raw images on `image_raw`.
   - Uses `rclcpp::SensorDataQoS` for low-latency image streaming.

2. `InferenceComponent`
   - Subscribes to `image_raw`.
   - Loads an ONNX model from `share/vision/models/model.onnx`.
   - Publishes detection results on `detections` and optional debug images on `detections_debug`.

3. `KinectComponent`
   - Interfaces with Kinect hardware through `libfreenect`.
   - Publishes:
     - `kinect/rgb/image_raw`
     - `kinect/depth/image_raw`
     - `kinect/depth/camera_info`
   - Uses a fixed camera intrinsics model and sets `frame_id` to `<camera_name>_link`.

4. `TargetLocatorComponent`
   - Subscribes to detections, depth image, and camera info.
   - Uses `message_filters::ApproximateTime` to synchronize these 3 streams.
   - Computes a 3D pose for the first valid detection and publishes it on `target_pose`.

## Launching

- `vision/launch/kinect.launch.py` starts the Kinect, inference, and target locator components in a multi-threaded component container (`component_container_mt`).
- `KinectComponent` publishes:
  - `kinect/rgb/image_raw`
  - `kinect/depth/image_raw`
  - `kinect/depth/camera_info`
- `InferenceComponent` expects image input on `image_raw` and produces detections used by `TargetLocatorComponent`.
- `TargetLocatorComponent` requires both depth and camera info topics to compute accurate 3D target poses.

## Communication graph

- `CameraComponent` → `image_raw` → `InferenceComponent`
- `InferenceComponent` → `detections` → `TargetLocatorComponent`
- `KinectComponent` → `kinect/depth/image_raw` and `kinect/depth/camera_info` → `TargetLocatorComponent`
- `TargetLocatorComponent` → `target_pose`

## Synchronization strategy

- `TargetLocatorComponent` uses `message_filters::Synchronizer<ApproximateTime>` with a queue size of 10.
- It aligns `detections`, `depth`, and `camera_info` streams approximately rather than strictly.
- This means the package tolerates small timestamp differences between image processing and depth capture, but it assumes the messages are close enough temporally to represent the same scene.

## Frame conventions

- `CameraComponent` sets image header `frame_id` to the configured `camera_name` parameter (default `cam_1`).
- `KinectComponent` sets `frame_id` to `kinect_link` by default.
- `TargetLocatorComponent` reuses the depth message frame for published `target_pose.header.frame_id`.
- The computed target pose is expressed in the Kinect camera frame, not in world or robot body coordinates.

## Hidden coupling points

- `InferenceComponent` expects the input topic to be exactly `image_raw` unless remapped.
- `TargetLocatorComponent` expects topics exactly named `detections`, `kinect/depth/image_raw`, and `kinect/depth/camera_info`.
- `gazebo_bringup.launch.py` remaps those topics to Gazebo's bridged camera topics, creating the only bridge between simulation and the vision stack.
- The package assumes the ONNX runtime is installed under `/opt/onnxruntime/onnxruntime-linux-x64-gpu-1.17.1`; changing that path requires CMake updates.
- `KinectComponent` assumes depth images are either `CV_16UC1` or `CV_32FC1`; other encodings are ignored.
- `target_pose` is published with a neutral quaternion (`w=1`), so orientation is intentionally fixed.

## Behavioral constraints

- If remapping the image or depth topic names, update both the inference and target locator components and any launch remaps.
- If changing the depth image encoding or camera intrinsics, update the `TargetLocatorComponent` depth conversion logic.
- Keep the hardcoded ONNX provider path in sync with installed runtime versions, or parameterize it before changing.
- Do not assume accurate pose output if the depth camera is not synchronized with detection frames; the current pipeline uses approximate time.

## Performance assumptions

- `image_raw` is expected to be a high-rate sensor stream, so a sensor QoS profile is used.
- Inference is designed for GPU acceleration but will fall back if CUDA provider initialization fails.
- Debug visualization is optional and should be disabled for production to reduce message bandwidth.
- The inference input tensor is assumed to be 640×640 and the model outputs are mapped to detection boxes on that grid.

## Useful details for AI context

- This package is the perception bridge from raw cameras to 3D target poses.
- It is responsible for the exact topic path and timestamp alignment used by higher-level planners.
- Changes in topic names, encoding, or frame IDs in this package must be coordinated with launch remaps and the rest of the system.
