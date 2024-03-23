#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "nav_2d_msgs/Point2D.h"

enum Type
{
   VISION_CONTROL = 0,
   YAW_CONTROL,
   SONAR_CONTROL,
   LIDAR_CONTROL
};

geometry_msgs::Twist cmd_vel_line;
geometry_msgs::Twist cmd_vel_yaw;
geometry_msgs::Twist cmd_vel_sonar;
geometry_msgs::Twist cmd_vel_rqt_steering;

int type_cmd_vel;
void Type_cmd_vel_Callback(const std_msgs::Int8::ConstPtr &msg)
{
    type_cmd_vel = msg->data;
}


void VISION_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_line = *msg;
}


void YAW_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_yaw = *msg;
}


void SONAR_Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
   cmd_vel_sonar = *msg;
}


int main(int argc, char **argv)
{
    int count = 0;
    ros::init(argc, argv, "cmd_vel_mux");
    ros::NodeHandle n;

    std::string line_topic  = "/cmd_vel/line";
    std::string yaw_topic   = "/cmd_vel/yaw"; 
    std::string sonar_topic = "/cmd_vel/sonar";
    std::string mux_topic   = "/ackermann_steering_controller/cmd_vel";
    std::string type_cmd_vel_topic = "/type_cmd_vel";

    ros::param::get("~line_topic",  line_topic);
    ros::param::get("~yaw_topic",   yaw_topic);
    ros::param::get("~sonar_topic", sonar_topic);
    ros::param::get("~mux_topic",   mux_topic);
    ros::param::get("~type_cmd_vel", type_cmd_vel_topic);
    
    ros::Subscriber line_sub  = n.subscribe(line_topic,  10, VISION_Callback);
    ros::Subscriber yaw_sub   = n.subscribe(yaw_topic,   10, YAW_Callback);
    ros::Subscriber sonar_sub = n.subscribe(sonar_topic, 10, SONAR_Callback);
    ros::Subscriber type_cmd_vel_sub = n.subscribe(type_cmd_vel_topic, 10, Type_cmd_vel_Callback);
    
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>(mux_topic , 1);
    
    ros::Rate loop_rate(100.0);//50 이상 조절할것
    
    while (ros::ok())
    {
      switch(type_cmd_vel)
      {
         case VISION_CONTROL:
            printf("vision\n");
            cmd_vel_rqt_steering = cmd_vel_line;
            cmd_vel_pub.publish(cmd_vel_rqt_steering);
            break;
         case YAW_CONTROL:
            printf("yaw\n");
            cmd_vel_rqt_steering = cmd_vel_yaw;
            cmd_vel_pub.publish(cmd_vel_rqt_steering);
            break;
         case SONAR_CONTROL:
            printf("sonar\n");
            cmd_vel_rqt_steering = cmd_vel_sonar;
            cmd_vel_pub.publish(cmd_vel_rqt_steering);
            break;
      }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
