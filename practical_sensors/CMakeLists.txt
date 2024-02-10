cmake_minimum_required(VERSION 3.0.2)
project(practical_sensors)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES practical_sensors
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_practical_sensors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

INCLUDE_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(/usr/local/lib)

add_executable(tick_publisher src/tick_publisher.cpp)
target_link_libraries(tick_publisher libpigpiod_if2.so ${catkin_LIBRARIES})

add_library(encoder src/encoder.cpp) 
add_dependencies(encoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(encoder ${catkin_LIBRARIES}) 

add_executable(encoder_node nodes/encoder_node.cpp) 
add_dependencies(encoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(encoder_node libpigpiod_if2.so ${catkin_LIBRARIES} encoder) 

