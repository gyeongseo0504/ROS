#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/MagneticField.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <numeric>
#include <string>
#include <functional>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    tf2::Quaternion quat;
    tf2::fromMsg(imu_msg -> orientation, quat);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    
    roll  *= 180.0 / M_PI;
    pitch *= 180.0 / M_PI;
    yaw   *= 180.0 / M_PI;
    
//ROS_INFO("Roll: %.2f,  Pitch: %.2f  Yaw: %.2f \n", roll,pitch,yaw);
			
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
   double x = msg->magnetic_field.x;
   double y = msg->magnetic_field.y;
        
   // 방위각 계산 (라디안)
   double azimuth = atan2(y, x);

   // 라디안을 도로 변환
   double azimuth_deg = azimuth * 180.0 / M_PI;
   
   // 0-360도 범위로 조정
   if(azimuth_deg < 0)
   {
     azimuth_deg += 360.0;
    }
     ROS_INFO("Azimuth: %.2f degrees", azimuth_deg);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "imu_rpy");

	ros::NodeHandle nh;  
	
    std::string imu_topic 		= "/handsfree/imu";
    std::string mag_topic	    = "/handsfree/mag";
    
    ros::param::get("~imu_topic", 		imu_topic);
    ros::param::get("~mag_topic", 		mag_topic);
    
    ros::Subscriber sub_imu_pos          = nh.subscribe(imu_topic, 1, imuCallback);
	ros::Subscriber sub_mag				 = nh.subscribe(mag_topic, 1, magCallback);
   
    ros::Rate rate(30);
    geometry_msgs::Twist cmd_vel_out;
    while (ros::ok())
    {      

        ros::spinOnce();
        rate.sleep();      
    }
   
}

