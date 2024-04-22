cmake_minimum_required(VERSION 3.0.2)
project(psolqr_local_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  nav_core
  roscpp
  std_msgs
  visualization_msgs
  tf
  navfn
  base_local_planner
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  nav_core
  roscpp
  std_msgs
  visualization_msgs
  tf
  navfn
  base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pso_trajectory.cpp
  src/pso_optimizer.cpp
  src/lqr_controller.cpp
  src/psolqr_planner.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES psolqr_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
