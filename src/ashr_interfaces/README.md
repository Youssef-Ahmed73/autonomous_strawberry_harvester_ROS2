# ashr_interfaces

## Purpose

`ashr_interfaces` is the central repository for all custom ROS 2 Interface Definition Language (IDL) files used in the ASHR (Autonomous Strawberry Harvester Robot) project. It provides standardized data structures, enabling seamless communication between the autonomy, perception, and control subsystems without creating circular dependencies.

## System architecture

This package acts as a pure structural dependency. It contains no executable nodes or logic. It uses `rosidl_default_generators` to compile `.msg`, `.srv`, and `.action` files into C++ headers and Python modules usable by other ROS 2 packages in the workspace.

## Provided Interfaces

### Services (`srv/`)

* **`CheckRipeness.srv`**
  * **Role:** Synchronous, on-demand ripeness evaluation.
  * **Request:** (Empty)
  * **Response:** * `bool is_ripe`: Final threshold-evaluated boolean indicating if the target is ready for harvest.
    * `float32 ripeness_percentage`: The calculated confidence/percentage of the target's ripeness.
  * **Usage:** Called by the autonomy state machine (`ashr_autonomy`) when the robot end-effector has engulfed a target. Serviced by the `vision` package (`ClassicalVisionComponent`) to return a multi-frame averaged ripeness score from the internal gripper cameras.