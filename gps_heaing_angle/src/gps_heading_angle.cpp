#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <math.h>

#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct utmData
{
    double East = 0.0;
    double North = 0.0;
};

utmData gps1_utm_data, gps2_utm_data;
int gps1_fix = 0;
int gps2_fix = 0;

double heading_angle_radian_old = 0.0;
double heading_angle_degree_old = 0.0;
bool heading_angle_value = false;

void gps1_fix_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    gps1_fix = gps_msg->status.status;
   
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    {
        ROS_ERROR("No fix for GPS1.");
    }
    else if(gps_msg->status.status == 1)
    {
        ROS_DEBUG_THROTTLE(60,"GPS1 float.");
    }
    else if(gps_msg->status.status == 2)
    {
        ROS_INFO("GPS1 FIX.");
    }
}
 
void gps2_fix_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    gps2_fix = gps_msg->status.status;
   
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX)
    {
        ROS_ERROR("No fix for GPS2.");
    }
    else if(gps_msg->status.status == 1)
    {
        ROS_DEBUG_THROTTLE(60,"GPS2 float.");
    }
    else if(gps_msg->status.status == 2)
    {
        ROS_INFO("GPS2 FIX.");
    }
}

void gps1_utm_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    gps1_utm_data.East = msg->x;
    gps1_utm_data.North = msg->y;
}

void gps2_utm_Callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    gps2_utm_data.East = msg->x;
    gps2_utm_data.North = msg->y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_heading_angle_node");
    ros::NodeHandle n;

    std::string gps1_fix_topic = "/gps1/fix";
    std::string gps2_fix_topic = "/gps2/fix";
    std::string gps1_utm_pub_topic = "/gps1/utm";
    std::string gps2_utm_pub_topic = "/gps2/utm";
    std::string gps_heading_angle_radian_topic = "/gps_heading_angle_radian";
    std::string gps_heading_angle_degree_topic = "/gps_heading_angle_degree";
    std::string gps_heading_angle_enable_topic = "/gps_heading_angle_enable";
   
    ros::param::get("~gps1_fix_topic", gps1_fix_topic);
    ros::param::get("~gps2_fix_topic", gps2_fix_topic);
    ros::param::get("~gps1_utm_pub_topic", gps1_utm_pub_topic);
    ros::param::get("~gps2_utm_pub_topic", gps2_utm_pub_topic);
    ros::param::get("~gps_heading_angle_radian_topic", gps_heading_angle_radian_topic);
    ros::param::get("~gps_heading_angle_degree_topic", gps_heading_angle_degree_topic);
    ros::param::get("~gps_heading_angle_enable_topic", gps_heading_angle_enable_topic);

    ros::Subscriber sub_gps1_fix = n.subscribe(gps1_fix_topic, 1, gps1_fix_check_Callback);
    ros::Subscriber sub_gps2_fix = n.subscribe(gps2_fix_topic, 1, gps2_fix_check_Callback);
    ros::Subscriber sub_gps1_utm = n.subscribe(gps1_utm_pub_topic, 1, gps1_utm_Callback);
    ros::Subscriber sub_gps2_utm = n.subscribe(gps2_utm_pub_topic, 1, gps2_utm_Callback);
   
    ros::Publisher pub_heading_radian = n.advertise<std_msgs::Float32>(gps_heading_angle_radian_topic, 1);
    ros::Publisher pub_heading_degree = n.advertise<std_msgs::Float32>(gps_heading_angle_degree_topic, 1);
    ros::Publisher pub_heading_enable = n.advertise<std_msgs::Bool>(gps_heading_angle_enable_topic, 1);
       
    ros::Rate loop_rate(25.0);
    std_msgs::Float32 heading_radian_msg;
    std_msgs::Float32 heading_degree_msg;
    std_msgs::Bool heading_enable_msg;

    while (ros::ok())
    {
        double diff_x = gps1_utm_data.East - gps2_utm_data.East;
        double diff_y = gps1_utm_data.North - gps2_utm_data.North;
               
        if((gps1_fix == 2) && (gps2_fix == 2))  
        {
			printf("GPS1 Fix: %d  GPS2 Fix: %d\n", gps1_fix, gps2_fix);

            double h_angle = atan2(-diff_x, diff_y);
           
            double h_angle_deg = RAD2DEG(h_angle);
            if (h_angle_deg > 180.0) h_angle_deg -= 360.0;
            if (h_angle_deg < -180.0) h_angle_deg += 360.0;
           
            heading_angle_radian_old = h_angle;
            heading_angle_degree_old = h_angle_deg;
            heading_angle_value = true;

            heading_radian_msg.data = h_angle;
            heading_degree_msg.data = h_angle_deg;
            heading_enable_msg.data = true;  
           
            printf("New Heading Angle: %6.2f degrees\n", h_angle_deg);
        }
        else
        {
            heading_enable_msg.data = false;
            printf("Using previous heading angle\n");
        }

        if (heading_angle_value)
        {
            heading_radian_msg.data = heading_angle_radian_old;
            heading_degree_msg.data = heading_angle_degree_old;
        }
        else
        {
            heading_radian_msg.data = 0.0;
            heading_degree_msg.data = 0.0;
            printf("No valid heading angle available yet\n");
        }
       
        pub_heading_radian.publish(heading_radian_msg);
        pub_heading_degree.publish(heading_degree_msg);
        pub_heading_enable.publish(heading_enable_msg);
       
        printf("Published Heading Angle: %6.2f degrees\n\n", heading_degree_msg.data);
       
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
