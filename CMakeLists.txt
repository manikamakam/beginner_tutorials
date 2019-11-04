cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

cmake_policy(SET CMP0046 OLD)

## Compile as C++11, supported in ROS Kinetic
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(FILES changeString.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
