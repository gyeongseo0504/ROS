#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_range = 0.0;

void Sonar_FrontCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    front_range = msg->range;
    
    if (front_range <= 2.0)
    {
        ROS_INFO("Sonar Range: [%f] - Stopping", front_range);
    }
    else
    {
        ROS_INFO("Sonar Range: [%f]", front_range);
    }
}

int main(int argc, char** argv)
{
    int count = 0;

    ros::init(argc, argv, "AEB_system");
    ros::NodeHandle n;
    
    geometry_msgs::Twist msgs_cmd;
    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("sensor/sonar0", 1000, Sonar_FrontCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();  

        if (front_range <= 2.0)
        {
            msgs_cmd.linear.x = 0;
        }
        else
        {
            msgs_cmd.linear.x = 0.5;
        }

        pub_cmd_vel.publish(msgs_cmd);
        loop_rate.sleep();  
        ++count;
    }

    return 0;
}
