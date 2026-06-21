# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment Setup

Always run before any ROS2 commands:
```bash
source /home/leo/proyecto_G/install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/leo/proyecto_G/world/models:${GZ_SIM_RESOURCE_PATH:-}"
```

## Build Commands

```bash
# Full workspace build (required after changing URDF/xacro or deps)
colcon build --symlink-install

# Fast rebuild of main package only (Python changes with --symlink-install often don't need rebuild)
colcon build --packages-select rosa_summit --symlink-install
```

There is no test suite or lint configuration — the package uses `ament_python` with no C++ code.

## Running the System

```bash
# Launch simulation with preloaded map (for navigation)
ros2 launch rosa_summit summit.launch.py

# Launch simulation with SLAM (for mapping + exploration)
ros2 launch rosa_summit summit.launch.py slam:=True

# Run the LLM agent (separate terminal, after simulation is up)
ros2 run rosa_summit rosa_summit

# Manual teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/summit/cmd_vel
```

## Architecture

This is a ROS2 (Humble) workspace controlling a **Summit XL robot with UR arm** in **Ignition Gazebo**, operated by an LLM agent (Claude Sonnet or Ollama) via the [ROSA framework](https://github.com/nasa-jpl/rosa).

### Key Packages
- **`rosa_summit/`** — Main package: LLM agent node with 8 robot control tools
- **`src/icclab_summit_xl/`** — Robot URDF/xacro, Nav2 config, simulation launch files
- **`src/sum/summit_xl_description/`** — Robot description (URDF/xacro), plugin configuration
- **`src/m-explore-ros2/`** — Frontier-based autonomous exploration (explore_lite)

### Motion Control
The robot uses **`ignition-gazebo-velocity-control-system`** (VelocityControl) instead of DiffDrive or gz_ros2_control for the base. This was chosen because the omni-wheels don't have sufficient traction with other controllers.

Topic chain for velocity commands:
```
ROS2 /summit/cmd_vel → relay node → /model/summit/cmd_vel (Ignition)
```

### Bridge Architecture
`ros_gz_bridge parameter_bridge` (starts at t=20s) connects Ignition ↔ ROS2:

| Ignition Topic | ROS2 Topic | Direction |
|---|---|---|
| `model/summit/cmd_vel` | `/model/summit/cmd_vel` | bidirectional |
| `model/summit/odom` | `/model/summit/odom` | Ignition→ROS2 |
| `model/summit/joint_states` | `/model/summit/joint_states` | Ignition→ROS2 |
| `tf` / `tf_static` | `/tf` / `/tf_static` | bidirectional |
| `clock` | `/clock` | Ignition→ROS2 |
| `summit/merged_laser_scan` | `/summit/merged_laser_scan` | Ignition→ROS2 |

Additional relays (t=21s):
- `/model/summit/odom` → `/summit/odom` (Nav2 expects this topic)
- `/tf` → `/summit/tf` (Nav2 in `/summit` namespace remaps `/tf` → `tf` → `/summit/tf`)

### TF Tree
- `map` → `odom`: published by **slam_toolbox** (when `slam:=True`) or AMCL
- `odom` → `base_footprint`: published by **OdometryPublisher** Ignition plugin (via bridge + relay)
- `base_footprint` → joints: published by **robot_state_publisher** from URDF

### Nav2 Namespace
All Nav2 nodes run under `/summit` namespace. Because of `use_namespace: True`, Nav2 remaps `/tf` → `tf` (relative), resolving to `/summit/tf`. This is why the `/tf` → `/summit/tf` relay is critical.

### LLM Agent Tools (rosa_summit.py)
The agent has 8 tools: `send_vel`, `stop`, `toggle_auto_exploration`, `navigate_to_pose`, `navigate_relative`, `save_map`, `list_saved_maps`, `navigate_to_location_by_name`.

Hardcoded named locations (in `small_house.world` coordinate frame): gym, kitchen, living room, office, bedroom.

### API Key
Located at `~/rap/Gruppe2/api-key.txt`. Model: `claude-sonnet-4-6`.

### Sensor
Both front and rear lidars publish merged to `/summit/merged_laser_scan` (Ignition topic), bridged to ROS2. Type: `gpu_lidar` with `<ray>` format (Ignition Fortress).

### Known Fragilities
- **arm_controller spawner** (t=25s) may time out — non-critical, arm defaults to limp state
- **Nav2 params** use `<robot_namespace>` placeholder strings — nav2_bringup substitutes these automatically when `use_namespace: True`
- The `ros2_laser_scan_merger` package is optional; Nav2 launch skips it if not found
- `VelocityControl` does not publish real wheel odometry; odometry comes from the `OdometryPublisher` Ignition physics plugin
