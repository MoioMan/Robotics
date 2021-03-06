#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/tf.h"

#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


#define RPM_TO_RADS(x)      x * (2 * M_PI) / 60 // 1 (rpm) : 2pi / 60 (rad/s) = n (rpm) : x (rad/s)
#define ERR_THRESHOLD       1e-8
#define EQUAL_WHEEL_SPEED_THRESHOLD 10e-2       // threshold under which the motion can be considered linear

using namespace geometry_msgs;
using namespace robotics_hw1;


static double WHEEL_RADIUS;
static double GEAR_RATIO;
static bool use_fixed_gear_ratio = false;


typedef message_filters::sync_policies
    ::ApproximateTime<MotorSpeed, MotorSpeed, MotorSpeed, MotorSpeed, nav_msgs::Odometry> MotorSyncPolicy;

void onMotorMessagesSync(const MotorSpeedConstPtr& msg_fl, 
                             const MotorSpeedConstPtr& msg_fr,
                             const MotorSpeedConstPtr& msg_bl,
                             const MotorSpeedConstPtr& msg_br,
                             const nav_msgs::Odometry::ConstPtr& msg) 
{
    static double app_baseline = 0.0;
    static double gear_ratio;
    static int count_gear = 0;
    static int count_base = 0;

    double rpm_fl, rpm_fr, rpm_bl, rpm_br;

    rpm_fl = RPM_TO_RADS(msg_fl->rpm);
    rpm_fr = RPM_TO_RADS(msg_fr->rpm);
    rpm_bl = RPM_TO_RADS(msg_bl->rpm);
    rpm_br = RPM_TO_RADS(msg_br->rpm);
    
    double w_z = msg->twist.twist.angular.z;
    double v_x = msg->twist.twist.linear.x;

    double rpm_l =  - (rpm_fl + rpm_bl) / 2;
    double rpm_r = (rpm_fr + rpm_br) / 2;
    double new_gear_ratio = 0;

    if(abs(rpm_r+rpm_l) > ERR_THRESHOLD)
    {
        new_gear_ratio = 2 * v_x / (WHEEL_RADIUS*(rpm_r + rpm_l));
        gear_ratio = (gear_ratio * count_gear + abs(new_gear_ratio)) / (count_gear+1);      // Incremental mean of gear ratio

        count_gear ++;
	}

    // If specified, use a fixed gear ratio (from rosparam); otherwise use the one which is currently computed
    double gear_ratio_to_use = use_fixed_gear_ratio ? GEAR_RATIO : gear_ratio;  
	double w_l = rpm_l * gear_ratio_to_use;
    double w_r = rpm_r * gear_ratio_to_use;

    double v_left = w_l * WHEEL_RADIUS;
    double v_right = w_r * WHEEL_RADIUS;


	if(abs(w_z) > ERR_THRESHOLD && abs(v_right - v_left) > EQUAL_WHEEL_SPEED_THRESHOLD)
	{
        double new_app_baseline = (v_right - v_left) / w_z;            
        app_baseline = (app_baseline*count_base + abs(new_app_baseline))/(count_base+1);    // Incremental mean of apparent baseline

        count_base++; 
    } 
    
	
	ROS_INFO("app: %f", app_baseline);
	ROS_INFO("gear: %f", gear_ratio_to_use);
}

int main(int argc, char **argv)
{  	
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

    // Read arg param
    use_fixed_gear_ratio = false;
    if (argc == 2)
        use_fixed_gear_ratio = atoi(argv[1]) == 1;

    ROS_INFO(use_fixed_gear_ratio ? "Using fixed gear ratio" : "Using online gear ratio");

    // Read rosparams
    bool ret = n.getParam("/gearRatio", GEAR_RATIO);    
    if (!ret)
    {
        ROS_INFO("No gear ratio found");
        use_fixed_gear_ratio = false;
    }
    ret = n.getParam("/wheelRadius", WHEEL_RADIUS);
    if (!ret)
    {
        ROS_ERROR("Unable to get wheelRadius parameter!");
        return -1;        
    }

	message_filters::Subscriber<nav_msgs::Odometry> sub_base(n, "/scout_odom", 1);
	message_filters::Subscriber<MotorSpeed> sub_fl(n, "/motor_speed_fl", 5); // front left topic
    message_filters::Subscriber<MotorSpeed> sub_fr(n, "/motor_speed_fr", 1); // front right topic
    message_filters::Subscriber<MotorSpeed> sub_bl(n, "/motor_speed_rl", 1); // front right topic
    message_filters::Subscriber<MotorSpeed> sub_br(n, "/motor_speed_rr", 1); // front right topic
        
    // Synchronizer using approximate policy, and an instance of it
    message_filters::Synchronizer<MotorSyncPolicy> sync(MotorSyncPolicy(25), sub_fl, sub_fr, sub_bl, sub_br, sub_base);
    sync.registerCallback(boost::bind(&onMotorMessagesSync, _1, _2, _3, _4, _5));
	
	ros::spin();
    return 0;
}

