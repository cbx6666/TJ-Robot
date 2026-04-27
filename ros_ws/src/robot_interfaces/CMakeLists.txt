cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
option(ROBOT_INTERFACES_ENABLE_ROSIDL "Enable rosidl message generation" OFF)

if(ROBOT_INTERFACES_ENABLE_ROSIDL)
  find_package(rosidl_default_generators REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/NodeStatus.msg"
    "msg/TaskStatus.msg"
    "msg/VoiceCommand.msg"
    "msg/TaskGoal.msg"
    "srv/ManipulationCommand.srv"
    DEPENDENCIES builtin_interfaces
  )
  ament_export_dependencies(rosidl_default_runtime)
else()
  message(STATUS "robot_interfaces: rosidl generation disabled (ROBOT_INTERFACES_ENABLE_ROSIDL=OFF)")
endif()

ament_package()
