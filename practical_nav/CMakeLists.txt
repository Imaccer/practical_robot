cmake_minimum_required(VERSION 3.0.2)
project(practical_nav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES practical_nav
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

add_library(path_planner src/path_planner_node.cpp)
add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner ${catkin_LIBRARIES})

add_executable(simple_diff_drive src/simple_diff_drive.cpp)
target_link_libraries(simple_diff_drive libpigpiod_if2.so ${catkin_LIBRARIES})

add_executable(simple_drive_controller src/simple_drive_controller.cpp)
target_link_libraries(simple_drive_controller ${catkin_LIBRARIES})

add_executable(ekf_drive_controller src/ekf_drive_controller.cpp)
target_link_libraries(ekf_drive_controller ${catkin_LIBRARIES})

add_executable(path_planner_node src/path_planner_node.cpp)
#target_link_libraries(path_planner ${catkin_LIBRARIES})
add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS}) 
target_link_libraries(path_planner_node ${catkin_LIBRARIES} path_planner)

add_library(differential_drive_robot src/differential_drive_robot.cpp) 
add_dependencies(differential_drive_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(differential_drive_robot libpigpiod_if2.so ${catkin_LIBRARIES}) 

add_executable(differential_drive_node nodes/differential_drive_node.cpp) 
add_dependencies(differential_drive_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(differential_drive_node libpigpiod_if2.so
  ${catkin_LIBRARIES} differential_drive_robot) 

