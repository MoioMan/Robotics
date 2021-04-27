#include "ros/ros.h"
#include "std_msgs/String.h"
#include "project1_skid/resetToPose.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset");

    ros::NodeHandle n;
    ros::ServiceClient client_reset_to_pose = n.serviceClient<project1_skid::resetToPose>("resetToPose");
    project1_skid::resetToPose srv_rst_pose;
    
    //srv_rst_pose.request.x = atof(argv[1]);
    //srv_rst_pose.request.y = atof(argv[2]);
    //srv_rst_pose.request.theta = atof(argv[3]);
    while(true)
    {
    std::cout << "Insert x, y, theta: ";
    std::cin >> srv_rst_pose.request.x;
    std::cin >> srv_rst_pose.request.y;
    std::cin >> srv_rst_pose.request.theta;
  

    if(client_reset_to_pose.call(srv_rst_pose))
        {
            
                ROS_INFO("OK %i",(int)srv_rst_pose.response.result);
        }
        else
        {
            ROS_ERROR("NON VA BENE");
            return 1;
        }


    }
    return 0;
}