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
int index = 0;

struct CMD_DATA
{
	int speed;
	int steer;	
};

struct CMD_DATA* parking_cmd;
void Cmd_Vel_Callback(const geometry_msgs::Twist& msg)
{
   parking_steer_data = (int)(msg.angular.z);
   parking_speed_data = (int)(msg.linear.x);
}
int count_waypoint(void)
{
	char line[2500];
	int count = 0;
	FILE *fp;
	fp= fopen("//home//chan//henes_catkin_ws//text//parking_data.txt","r");
	
	if(fp == NULL)
	{
		ROS_INFO("Parking_data does not exit!");
	}
    else
    {
	    while(fgets(line, sizeof(line), fp)!= NULL)
	    {	
			    count++;	    
	    } 
    }
	fclose(fp);
	return count;
}
void Parking_data_read_to_file()
{
	index = count_waypoint();
	parking_cmd = (struct CMD_DATA*)malloc(sizeof(struct CMD_DATA)*index);
	
	if(fp == NULL)
	{
		ROS_INFO("Parking_data does not exit!");
	}
	else 
	{
		for(int i = 0; i< index; i++)
		{
			fscanf(fp,"%d %d", &parking_cmd[i].speed, &parking_cmd[i].steer);
		}
		ROS_INFO("Parking_data Number %d",index);
		for(int i=0; i<index; i++)
	    {
			ROS_INFO("Parking_data-%d : [%d] [%d]",i , parking_cmd[i].speed , parking_cmd[i].steer);
	    }
	    fclose(fp);
	}
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
	  
	for(int i=0; i<index; i++)
	{
	     stear.data = parking_cmd[i].speed
	     speed.data = parking_cmd[i].steer
	     car_speed_pub.publish()
	     car_steer_pub.publish()
	}
    parking_data_write_to_file();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

