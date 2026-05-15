cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/coverage_monitor.py
  scripts/patrol_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/coverage_planner.py
  scripts/navigation_manager.py
  scripts/patrol_markers.py
  scripts/waypoint_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
