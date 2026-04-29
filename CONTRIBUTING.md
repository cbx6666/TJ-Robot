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

## RGBD + YOLO Focus

- This stage keeps RGBD robot simulation and YOLO recognition.
- Laser SLAM mapping is an active runtime path (for example via `scripts/run_mapping.sh`).
- Legacy Nav2/coverage patrol flows are not part of the current active runtime path.

## Data Outputs

Generated logs/results belong to `data/`:

- `data/logs`
- `data/results`

## Tests and CI

- Add unit tests for pure logic under package-local `test/`.
- CI workflow: `.github/workflows/ros2-ci.yml`.
- Runtime entry scripts under `scripts/` auto-source ROS/workspace setup when available (`common.sh`), so manual sourcing is not required before running `run_*.sh`.
- If running `colcon` directly in terminal, source ROS setup manually first.
- Local sanity flow:

```bash
cd ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to robot_bringup human_yolo_seg robot_tasks robot_interfaces robot_interaction robot_manipulation
colcon test --packages-select robot_tasks
colcon test-result --verbose
```

## Interface Contracts

- Message contracts live in `ros_ws/src/robot_interfaces/msg`.
- Service contracts live in `ros_ws/src/robot_interfaces/srv`.
- `robot_interfaces` uses optional rosidl generation:
  - OFF by default for compatibility in problematic non-ASCII paths.
  - ON in CI via `-DROBOT_INTERFACES_ENABLE_ROSIDL=ON`.
