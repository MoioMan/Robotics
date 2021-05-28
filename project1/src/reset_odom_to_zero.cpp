#include "ros/ros.h"
#include "std_msgs/String.h"
#include "project1_skid/resetToZero.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset");

    ros::NodeHandle n;
    ros::ServiceClient client_reset_to_zero = n.serviceClient<project1_skid::resetToZero>("resetToZero");
    project1_skid::resetToZero srv_rst_zero;
    

    if(client_reset_to_zero.call(srv_rst_zero))
    {
        ROS_INFO("OK %i",(int)srv_rst_zero.response.result);
    }
    else
    {
        ROS_ERROR("NON VA BENE");
        return 1;
    }

    return 0;
}
