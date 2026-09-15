cmake_minimum_required(VERSION 3.0.2)
project(alps_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/lio_tare_bridge.py
  scripts/navigation_guard.py
  scripts/goal_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
