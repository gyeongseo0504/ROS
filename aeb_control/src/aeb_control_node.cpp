#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

double front_range = 0.0;

void Sonar_FrontCallback(const sensor_msgs::Range::ConstPtr& msg)
{
   front_range = msg->range;   
}

int main(int argc, char **argv)
{
  int count = 0;
  
  ros::init(argc, argv, "AEB_system");
  geometry_msgs::Twist msg_cmd;  
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/range_front", 1000, Sonar_FrontCallback);
  ros::Publisher pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30);
  
  while (ros::ok())
  {
   ROS_INFO("Sonar Range: [%f]", front_range);
   
   if (front_range <= 1.0)
   {
      msg_cmd.linear.x = 0;
   } 
   else
   {
      msg_cmd.linear.x = 0.5;
   }
   
    pub_cmd_vel.publish(msg_cmd);
  
    ros::spinOnce();
    ++count;
  }
  return 0;
}
