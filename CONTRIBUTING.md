# Contributing Guide

## Source of Truth

- Runtime source code lives in `ros_ws/src`.
- Do not add parallel runtime logic at repository root.
- New features should be implemented in an existing ROS package, or a new package under `ros_ws/src`.

## Directory Conventions

- Nodes: `<pkg>/<pkg>/nodes`
- Offline tools / one-shot CLIs: `<pkg>/<pkg>/tools`
- Pure utilities / reusable logic: `<pkg>/<pkg>/utils`
- Runtime parameters: `robot_bringup/config/*`

## Nav2 Config Layering

Use layered config files instead of one giant YAML:

- Base: `ros_ws/src/robot_bringup/config/nav2/base.yaml`
- Profile overrides: `ros_ws/src/robot_bringup/config/nav2/profiles/*.yaml`

Launch accepts:

- `nav2_base_params`
- `nav2_profile_params`

## Data Outputs

Generated maps/logs/results belong to `data/`:

- `data/maps/baseline_raw`
- `data/maps/semantic_pre_strip`
- `data/maps/semantic_post_strip`
- `data/maps/semantic_overlays`
- `data/logs`
- `data/results`

## Tests and CI

- Add unit tests for pure logic under package-local `test/`.
- CI workflow: `.github/workflows/ros2-ci.yml`.
- Local sanity flow:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup robot_navigation human_yolo_seg robot_tasks robot_interfaces robot_interaction robot_manipulation
colcon test --packages-select robot_navigation robot_tasks
colcon test-result --verbose
```

## Interface Contracts

- Message contracts live in `ros_ws/src/robot_interfaces/msg`.
- Service contracts live in `ros_ws/src/robot_interfaces/srv`.
- `robot_interfaces` uses optional rosidl generation:
  - OFF by default for compatibility in problematic non-ASCII paths.
  - ON in CI via `-DROBOT_INTERFACES_ENABLE_ROSIDL=ON`.
