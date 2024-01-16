#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double range_front = 0.0;
double speed = 0.0;

void Sona_FrontCallback(const sensor_msgs::Range::ConstPtr& msg)
{
    range_front = msg->range; 
}

int main(int argc, char **argv)
{
    int count = 0;
    
    ros::init(argc, argv, "aeb_control"); 

    ros::NodeHandle n;

    ros::Subscriber range_front_sub = n.subscribe("/range_front", 1000, Sona_FrontCallback);

    ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30);  

    geometry_msgs::Twist msg_cmd;
    
    while (ros::ok()) 
    {
		ROS_INFO("Sonar Range: [%f]", range_front); 
		
        if (range_front < 1.0)
        {
            speed = 0.0;
            msg_cmd.linear.x = speed; 
        }
        else
        {
            speed = 0.9;
            msg_cmd.linear.x = speed; 
        }

        pub_cmd_vel.publish(msg_cmd); 

        ros::spinOnce();
        ++count;
    }
    return 0;
}
