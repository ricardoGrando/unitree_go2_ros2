# Robust multi-robot spawn patch v1.1

This patch keeps the existing Unitree Go2 model and controller configuration, but changes the multi-robot launch startup path.

## Changes

1. `ros_gz_sim/create` receives a namespace-specific generated URDF with `-file` instead of subscribing to `<namespace>/robot_description`.
   - `robot_state_publisher` is retained and continues publishing `robot_description`, TF, etc.
   - The Gazebo entity spawn no longer depends on ROS discovery timing for a one-shot description message.
2. Each namespace-specific xacro is generated once; the same XML is reused for robot_state_publisher and CHAMP, while Gazebo reads the corresponding temporary URDF file.
3. Controller-manager bootstrap timers are spread farther apart so joint-state and effort controller spawners from adjacent robots do not start at exactly the same wall-clock time.
4. The existing 8 s full-robot startup staggering is retained to limit Gazebo / ros2_control startup load.

No robot model, gait, controller type, spawn position, bridge topic, or ROS namespace was changed.
