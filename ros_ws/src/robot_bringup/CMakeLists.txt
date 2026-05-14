cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  world
  maps
  models
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/scan_rviz_relay.py DESTINATION lib/${PROJECT_NAME} RENAME scan_rviz_relay)

ament_package()

