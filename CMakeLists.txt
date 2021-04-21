cmake_minimum_required(VERSION 2.8.3)
project(project1_skid)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs message_generation)

# Custom Message : add & generation
add_message_files(
 FILES
 SkidOdometry.msg
 )
 
generate_messages(
   DEPENDENCIES
    nav_msgs
    std_msgs
 )
 

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## --- Add project nodes ---------------------------------------------
## add_executable([node-name] src/[mainfile].cpp)
## target_link_libraries([node-name] ${catkin_LIBRARIES})
## in case it use code generated for custom messages:
## add_dependencies([node-name] custom_messages_generate_messages_cpp)

add_executable(odometry_pub src/odometry_pub.cpp)
target_link_libraries(odometry_pub ${catkin_LIBRARIES})
add_dependencies(odometry_pub custom_messages_generate_messages_cpp)