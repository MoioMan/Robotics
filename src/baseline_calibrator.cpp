#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"

typedef struct {
	double m;
	double q;
} line;

static double previousTime;
static double previousTeta; 
static bool skipFirst = true;
static std::vector<line> lines;
static std::vector<double> baselines;
static double realBaseline = 0.583;

void addLine(const nav_msgs::Odometry::ConstPtr& msg){
	double vL = 0.0 ;
	double vR = 0.0 ;
	double vX = msg->twist.twist.linear.x;
	double teta = tf::getYaw(msg->pose.pose.orientation);
	double dotTeta;
	double deltaT;
	double currentTime = double(msg->header.stamp.toSec());
	double approxBaseline = 0.0;

	if(skipFirst)
	{
		previousTime = double(msg->header.stamp.toSec());
		previousTeta = tf::getYaw(msg->pose.pose.orientation);
		skipFirst =false;
		return;
	}

	deltaT = double(currentTime) - double(previousTime);
	dotTeta = (teta - previousTeta)/deltaT;
	vL = vX - sin(teta) * (realBaseline/2) * dotTeta;
	vR = vX + sin(teta) * (realBaseline/2) * dotTeta;
	approxBaseline = (vR - vL) / (msg->twist.twist.angular.z);
	baselines.push_back(approxBaseline);
	previousTeta = teta;
	previousTime = currentTime;
	ROS_INFO("%f",approxBaseline);

}

int main(int argc, char **argv){
  	
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
  	ros::Subscriber sub = n.subscribe("/scout_odom", 1000, addLine);

	ros::spin();

  return 0;
}

