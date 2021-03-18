cmake_minimum_required(VERSION 2.8.3)
project(SRBProject)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
  tf
  hardware_interface
  transmission_interface
  controller_manager
)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES trobo_control
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(HelloWorld src/HelloWorld.cpp )
target_link_libraries(HelloWorld ${catkin_LIBRARIES})

add_executable(SRBTestPublisher src/SRBTestPublisher.cpp )
target_link_libraries(SRBTestPublisher ${catkin_LIBRARIES})

add_executable(SRBTestSubscriber src/SRBTestSubscriber.cpp )
target_link_libraries(SRBTestSubscriber ${catkin_LIBRARIES})

add_executable(JoyToTwistPublisher src/JoyToTwistPublisher.cpp )
target_link_libraries(JoyToTwistPublisher ${catkin_LIBRARIES})


