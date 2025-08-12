#define DEBUG 0
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include <string.h>  
#include <unistd.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  

int parking_steer_data = 0;
int parking_speed_data = 0;

void Cmd_Vel_Callback(const geometry_msgs::Twist& msg)
{
   parking_steer_data = (int)(msg.angular.z);
   parking_speed_data = (int)(msg.linear.x);
}
void parking_data_write_to_file()
{
	FILE *fp;
	fp = fopen("//home//chan//henes_catkin_ws//text//parking_data.txt", "a");
	fprintf(fp, "%d %d\n", parking_speed_data, parking_steer_data);
	fclose(fp);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Car_Control");

  ros::NodeHandle n;  
  ros::Subscriber sub1 = n.subscribe("/cmd_vel_out", 10, &Cmd_Vel_Callback);
  //ros::Publisher car_control_pub1 = n.advertise<std_msgs::String>("Car_Control/SteerAngle_msgs", 10);
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
	  
    parking_data_write_to_file();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

