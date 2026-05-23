# ashr_simulation

## Purpose

`ashr_simulation` is the Gazebo/ROS 2 simulation asset package for the Probot project. It contains the Gazebo world, bridge configuration, and mesh resources used when the robot is brought up in simulation.

## System role

This package is a resource-only package that supports simulated execution. It does not contain nodes or algorithms itself. Its artifacts are consumed by `probot_bringup` when `use_gazebo` is enabled.

## Key contents

- `worlds/ashr_world.sdf` - primary Gazebo simulation world for the robot and environment
- `config/bridge.yaml` - ROS/Gazebo bridge configuration used by `ros_gz_bridge`
- `meshes/` - static geometry assets referenced by the URDF / Gazebo world
- `CMakeLists.txt` - installs simulation assets to `share/ashr_simulation`
- `package.xml` - declares the package as `ament_cmake` and identifies it as a simulation resource package

## Architecture and communication graph

- `probot_bringup.launch.py` loads this package only for the Gazebo path.
- The world file and bridge config are passed to `ros_gz_sim` and `ros_gz_bridge`.
- `bridge.yaml` is the coupling point between Gazebo entities and ROS 2 topics; changes here affect how world sensor topics, joint states, and actuators appear in ROS.
- The package sits at the simulation boundary: it is the asset layer that connects the Gazebo world to ROS via the bridge.

## Hidden coupling points

- `probot_bringup/launch/probot_bringup.launch.py` expects `ashr_world.sdf` and `bridge.yaml` to exist in this package share path.
- The launch file also depends on `IGN_GAZEBO_RESOURCE_PATH` to resolve Gazebo resources. If this package's mesh paths change, the environment variable build must be preserved.
- Mesh naming and world references are implicitly coupled to the robot description and Gazebo bridge topic names.

## Behavioral constraints

- Do not rename or move the world or bridge config without updating the bringup launch and any Gazebo-specific remappings.
- Maintain `bridge.yaml` topic mappings carefully; wrong mappings can break the simulated sensor and controller pipeline.
- Preserve the semantics of the simulated world relative to `probot_description` and the `probot_anno` model.

## Performance and assumptions

- Assumes Gazebo simulation will run with `ros_gz_sim` and `ros_gz_bridge` available.
- Assumes the environment can resolve package share paths using `ament_index_cpp`.
- Not performance-critical inside ROS itself, but the simulation assets must be present and correct for Gazebo to instantiate the robot and sensors.