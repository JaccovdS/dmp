cmake_minimum_required(VERSION 2.8.3)
project(dmp)

#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()

find_package(catkin REQUIRED COMPONENTS
eigen
roscpp
rospy
std_msgs
message_generation
)

## Declare ROS messages and services
add_message_files(
 FILES 
 DMPData.msg 
 DMPPoint.msg 
 DMPTraj.msg
 )

add_service_files(
 FILES 
 GetDMPPlan.srv 
 LearnDMPFromDemo.srv 
 SetActiveDMP.srv
 )

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS common_msgs image_common
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(dmp src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp)
target_link_libraries(dmp ${catkin_LIBRARIES})
add_dependencies(dmp dmp_generate_messages_cpp)

add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server dmp)
add_dependencies(dmp_server dmp_generate_messages_cpp)

add_executable(encode_test_trajectory src/encode_test_trajectory.cpp)
target_link_libraries(encode_test_trajectory ${catkin_LIBRARIES})
add_dependencies(encode_test_trajectory dmp_generate_messages_cpp)
