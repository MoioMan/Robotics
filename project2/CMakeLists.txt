cmake_minimum_required(VERSION 2.8.3)
project(project2_skid)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs tf )

# Custom Message : add & generation
 

## Declare a catkin package
catkin_package( CATKIN_DEPENDS )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## --- Add project nodes ---------------------------------------------
## add_executable([node-name] src/[mainfile].cpp)
## target_link_libraries([node-name] ${catkin_LIBRARIES})
## in case it use code generated for custom messages:
## add_dependencies([node-name] project1_skid_generate_messages_cpp)