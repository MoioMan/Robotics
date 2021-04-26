#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define RPM_TO_RADS(x)      x * (2 * M_PI) / 60 // 1 (rpm) : 2pi / 60 (rad/s) = n (rpm) : x (rad/s)

using namespace geometry_msgs;
using namespace robotics_hw1;

static double WHEEL_RADIUS;
static double GEAR_RATIO; // rpm_motor * GEAR_RATIO = rpm_wheel
static double APPARENT_BASELINE;

static ros::Publisher twistPublish;
static Twist estimateTwist(double omega_left, double omega_right);
static void publishTwist(Twist twist, ros::Time stamp);

typedef message_filters::sync_policies
    ::ApproximateTime<MotorSpeed, MotorSpeed, MotorSpeed, MotorSpeed> MotorSyncPolicy;


void onMotorMessagesSync(const MotorSpeedConstPtr& msg_fl, 
                             const MotorSpeedConstPtr& msg_fr,
                             const MotorSpeedConstPtr& msg_bl,
                             const MotorSpeedConstPtr& msg_br) 
{
    double w_fl, w_fr, w_bl, w_br;

    // Change sign for left omega
    w_fl = -RPM_TO_RADS(msg_fl->rpm * GEAR_RATIO);
    w_fr = RPM_TO_RADS(msg_fr->rpm * GEAR_RATIO);
    w_bl = -RPM_TO_RADS(msg_bl->rpm * GEAR_RATIO);
    w_br = RPM_TO_RADS(msg_br->rpm * GEAR_RATIO);
    
    // Odometry assumption => wheel on the same size should have same angular velocity
    // Use an avg of the two.

    // Compute robot linear & angular velocity
    Twist twist = estimateTwist((w_fl + w_bl) / 2, (w_fr + w_br) / 2);
    publishTwist(twist, msg_fl->header.stamp);
}

Twist estimateTwist(double omega_left, double omega_right)
{
    Twist twist;
    twist.linear.z = twist.angular.x = twist.angular.y = 0; // plane geometry
    
    double v_left = omega_left * WHEEL_RADIUS;
    double v_right = omega_right * WHEEL_RADIUS; 

    twist.linear.y = 0;
    twist.linear.x = (v_right + v_left) / 2;
    twist.angular.z = (v_right - v_left) / APPARENT_BASELINE;

    return twist;
}

void publishTwist(Twist twist, ros::Time stamp)
{
    static uint32_t id = 0;  // Stamped seq identifier
    id += 1;

    TwistStamped msg;
    msg.twist = twist;

    msg.header.frame_id = "base_link";
    msg.header.seq = id;
    msg.header.stamp = stamp;

    twistPublish.publish(msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "odometry_pub");
    ros::NodeHandle n;

    // read constant parameters
    std::string wheelRadius;  
    std::string app_Baseline;  
    std::string gearRatio;
    bool ret = n.getParam("/wheel_radius", wheelRadius);
    if (!ret)
    {
        ROS_ERROR("Unable to get wheel_radius parameter!");
        return -1;        
    }
    ret = n.getParam("/real_baseline", app_Baseline);    
    if (!ret)
    {
        ROS_ERROR("Unable to get app_baseline parameter!");
        return -1;        
    }
    ret = n.getParam("/gear_ratio", gearRatio);    
    if (!ret)
    {
        ROS_ERROR("Unable to get gear_ratio parameter!");
        return -1;        
    }

    WHEEL_RADIUS = std::stod(wheelRadius);
    APPARENT_BASELINE = std::stod(app_Baseline);
    GEAR_RATIO = std::stod(gearRatio);

	twistPublish = n.advertise<TwistStamped>("skid_twist", 500);	// Msg + (buffer)

    message_filters::Subscriber<MotorSpeed> sub_fl(n, "/motor_speed_fl", 1); // front left topic
    message_filters::Subscriber<MotorSpeed> sub_fr(n, "/motor_speed_fr", 1); // front right topic
    message_filters::Subscriber<MotorSpeed> sub_bl(n, "/motor_speed_bl", 1); // front right topic
    message_filters::Subscriber<MotorSpeed> sub_br(n, "/motor_speed_br", 1); // front right topic
        
    // Synchronizer using approximate policy, and an instance of it
    message_filters::Synchronizer<MotorSyncPolicy> sync(MotorSyncPolicy(25), sub_fl, sub_fr, sub_bl, sub_br);
    sync.registerCallback(boost::bind(&onMotorMessagesSync, _1, _2, _3, _4));

    ros::spin();
    return 0;
}