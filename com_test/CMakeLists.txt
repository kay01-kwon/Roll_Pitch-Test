cmake_minimum_required(VERSION 3.0.2)
project(com_test)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES com_test
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(com_test src/com_test.cpp)
target_link_libraries(com_test ${catkin_LIBRARIES})