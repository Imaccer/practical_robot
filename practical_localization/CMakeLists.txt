cmake_minimum_required(VERSION 3.0.2)
project(practical_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES practical_localization
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf2_ros
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

#add_executable(encoder_odom_publisher src/encoder_odom_publisher.cpp)
#target_link_libraries(encoder_odom_publisher ${catkin_LIBRARIES})

add_executable(manual_pose_and_goal_pub src/manual_pose_and_goal_pub.cpp)
target_link_libraries(manual_pose_and_goal_pub ${catkin_LIBRARIES})

add_executable(rviz_click_to_2d src/rviz_click_to_2d.cpp)
target_link_libraries(rviz_click_to_2d ${catkin_LIBRARIES})

add_executable(odom_plus src/odom_plus.cpp)
target_link_libraries(odom_plus ${catkin_LIBRARIES})

add_executable(tf_pub src/tf_pub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})

add_library(encoder_odom_publisher src/encoder_odom_publisher.cpp) 
add_dependencies(encoder_odom_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(encoder_odom_publisher ${catkin_LIBRARIES}) 

add_executable(encoder_odom_publisher_node nodes/encoder_odom_publisher_node.cpp) 
add_dependencies(encoder_odom_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(encoder_odom_publisher_node 
  ${catkin_LIBRARIES} encoder_odom_publisher) 


