#include "ros/ros.h"
#include "project1_skid/SkidOdometry.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace ros;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace tf2_ros;

class pub_sub {

public:
    TwistStamped message1;

private:
    NodeHandle n;

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double lastStamp = 0.0;

    Subscriber twistSub;
    Publisher odomPub; 
    TransformBroadcaster br;
    TransformStamped transformStamped;
    std::string integrationMethod = "Euler";

public:
    pub_sub() {
        twistSub = n.subscribe("/skid_twist", 1, &pub_sub::computeOdom, this);
        odomPub = n.advertise<project1_skid::SkidOdometry>("/odometry", 1);
    }

    void computeOdom(const TwistStamped::ConstPtr& msg) {
        double samplingTime;
        samplingTime = lastStamp == 0 ? 0.0 : msg->header.stamp.toSec() - lastStamp;
        lastStamp = msg->header.stamp.toSec();

        if (integrationMethod == "Euler") eulerIntegration(msg, samplingTime);
        else if (integrationMethod == "RungeKutta") rungeKuttaIntegration(msg, samplingTime);

        geometry_msgs::Quaternion qMsg;
        tf2::Quaternion q;

        q.setRPY(0, 0, theta);
        qMsg = tf2::toMsg(q);

        sendTransform(q);
        publishOdom(msg, qMsg);
    }

    void eulerIntegration(const TwistStamped::ConstPtr& msg, double samplingTime) {
        double linearVelocity = msg->twist.linear.x;
        double angularVelocity = msg->twist.angular.z;
        x+=linearVelocity*samplingTime*cos(theta);
        y+=linearVelocity*samplingTime*sin(theta);
        theta+=angularVelocity*samplingTime;
    }

    void rungeKuttaIntegration(const TwistStamped::ConstPtr& msg, double samplingTime) {
        double linearVelocity = msg->twist.linear.x;
        double angularVelocity = msg->twist.angular.z;
        x+=linearVelocity*samplingTime*cos(theta+(angularVelocity*samplingTime)/2);
        y+=linearVelocity*samplingTime*sin(theta+(angularVelocity*samplingTime)/2);
        theta+=angularVelocity*samplingTime;
    }

    void sendTransform(tf2::Quaternion q) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "robot";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
    }

    void publishOdom(const TwistStamped::ConstPtr& msg, geometry_msgs::Quaternion q) {
        double linearVelocity = msg->twist.linear.x;
        double angularVelocity = msg->twist.angular.z;
        
        nav_msgs::Odometry odom;
        odom.header.stamp=ros::Time::now();
        odom.header.frame_id="odom";

        odom.pose.pose.position.x=x;
        odom.pose.pose.position.y=y;
        odom.pose.pose.position.z=0.0;
        odom.pose.pose.orientation=q;

        odom.child_frame_id="robot";
        odom.twist.twist.linear.x=linearVelocity;
        odom.twist.twist.linear.y=0.0;
        odom.twist.twist.linear.z=0.0;

        odom.twist.twist.angular.x=0.0;
        odom.twist.twist.angular.y=0.0;
        odom.twist.twist.angular.z=angularVelocity;

        //publish custom odom
        project1_skid::SkidOdometry skidOdom;
        skidOdom.odom=odom;
        skidOdom.method.data = integrationMethod;
        odomPub.publish(skidOdom);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_pub");
  
    pub_sub my_pub_sub;
  
    ros::spin();
  
    return 0;
}