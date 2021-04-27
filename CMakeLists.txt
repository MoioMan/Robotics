cmake_minimum_required(VERSION 2.8.3)
project(project1_skid)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs message_generation robotics_hw1 message_filters tf dynamic_reconfigure)

# Custom Message : add & generation
add_message_files(
 FILES
 SkidOdometry.msg
 )

add_service_files(
 FILES
 resetToZero.srv
 resetToPose.srv
 )

generate_messages(
   DEPENDENCIES
    nav_msgs
    std_msgs
 )

generate_dynamic_reconfigure_options( 
   cfg/Parameters.cfg
)
 

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime robotics_hw1 message_filters)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## --- Add project nodes ---------------------------------------------
## add_executable([node-name] src/[mainfile].cpp)
## target_link_libraries([node-name] ${catkin_LIBRARIES})
## in case it use code generated for custom messages:
## add_dependencies([node-name] project1_skid_generate_messages_cpp)

add_executable(odometry_pub src/odometry_pub.cpp)
target_link_libraries(odometry_pub ${catkin_LIBRARIES})
add_dependencies(odometry_pub ${PROJECT_NAME}_gencfg)
add_dependencies(odometry_pub project1_skid_generate_messages_cpp)

add_executable(motor_synchronizer src/motor_synchronizer.cpp)
target_link_libraries(motor_synchronizer ${catkin_LIBRARIES})
add_dependencies(motor_synchronizer robotics_hw1_generate_messages_cpp)

add_executable(baseline_calibrator src/baseline_calibrator.cpp)
target_link_libraries(baseline_calibrator ${catkin_LIBRARIES})

add_executable(reset_odom_to_pose src/reset_odom_to_pose.cpp)
target_link_libraries(reset_odom_to_pose ${catkin_LIBRARIES})

add_executable(reset_odom_to_zero src/reset_odom_to_zero.cpp)
target_link_libraries(reset_odom_to_zero ${catkin_LIBRARIES})
