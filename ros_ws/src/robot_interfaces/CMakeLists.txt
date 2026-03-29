cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)

# 占位包：不在此处调用 rosidl_generate_interfaces，以便在 WSL + /mnt/e/... 含中文路径时
# colcon 仍能完成配置（该环境下 rosidl 曾把路径错解析为 /TJ-Robot/...，导致 NodeStatus.idl 找不到）。
#
# 需要生成自定义 msg/srv/action 时，请任选其一：
#   1) 将工作区放在仅 ASCII 的路径（推荐：~/tj_ws）；或
#   2) 在 Linux 原生分区（如 ~/）克隆本仓库后再 colcon build。
# 然后取消下面注释，并安装 rosidl 相关依赖（与官方接口包相同）。

# find_package(rosidl_default_generators REQUIRED)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/NodeStatus.msg"
# )
# ament_export_dependencies(rosidl_default_runtime)

ament_package()
