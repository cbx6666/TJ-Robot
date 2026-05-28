# shellcheck shell=bash
# 仿真 assist 栈共用环境（run_simulation.sh / run_full_system.sh 在设置 TB3_LOG_DIR 后 source）
# 一体 RGB-D：Gazebo 彩色 /camera/image_raw + /camera/camera_info，深度 /camera/depth/*

export TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-8}"
export TB3_CAMERA_ALWAYS_ON="${TB3_CAMERA_ALWAYS_ON:-1}"
export TB3_SIM_CAMERA_WIDTH="${TB3_SIM_CAMERA_WIDTH:-640}"
export TB3_SIM_CAMERA_HEIGHT="${TB3_SIM_CAMERA_HEIGHT:-480}"
export TB3_SIM_DEPTH_WIDTH="${TB3_SIM_DEPTH_WIDTH:-640}"
export TB3_SIM_DEPTH_HEIGHT="${TB3_SIM_DEPTH_HEIGHT:-480}"
export TB3_DEPTH_CLIP_FAR_M="${TB3_DEPTH_CLIP_FAR_M:-12.0}"
# 1=单传感器 depth 插件；0=旧双相机 tb3_depth_only
export TB3_SIM_UNIFIED_RGBD="${TB3_SIM_UNIFIED_RGBD:-1}"
export YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
export YOLO_CAMERA_INFO_TOPIC="${YOLO_CAMERA_INFO_TOPIC:-/camera/camera_info}"
export RGBD_DEPTH_IMAGE_TOPIC="${RGBD_DEPTH_IMAGE_TOPIC:-/camera/depth/image_raw}"
export RGBD_DEPTH_CAMERA_INFO_TOPIC="${RGBD_DEPTH_CAMERA_INFO_TOPIC:-/camera/depth/camera_info}"
export TB3_YOLO_DEPTH_REGISTER="${TB3_YOLO_DEPTH_REGISTER:-0}"
export TB3_YOLO_DEPTH_SAMPLE_STAT="${TB3_YOLO_DEPTH_SAMPLE_STAT:-min}"
export TB3_YOLO_DEPTH_RANGE_TO_OPTICAL_Z="${TB3_YOLO_DEPTH_RANGE_TO_OPTICAL_Z:-false}"
export TB3_GZSERVER_WAIT_SEC="${TB3_GZSERVER_WAIT_SEC:-120}"
export TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-1}"
