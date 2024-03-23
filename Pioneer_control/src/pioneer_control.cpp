#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

int mission_flag = 0;
double front_sonar = 0.0;
double find_line_center = 0.0;
double yaw_degree;
double roll, pitch, yaw;

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    front_sonar = msg->range;
}

void line_centroid_Callback(const std_msgs::Float64::ConstPtr &msg)
{
    find_line_center = msg->data;
}

void yaw_degree_Callback(const std_msgs::Float64::ConstPtr &msg)
{
    yaw_degree = msg->data;
}


int main(int argc, char **argv)
{
    int count = 0;
    
    ros::init(argc, argv, "pioneer_control");
    ros::NodeHandle nh;

    std::string type_cmd_vel_topic                 = "/type_cmd_vel";
    std::string target_yaw_degree_topic            = "/target_yaw/degree";
    std::string yaw_degree_topic                   = "/yaw_degree";
    std::string line_centroid_topic                = "/line_centroid";
    std::string Kp_line_topic                      = "/Kp_line";
    std::string front_sonar_topic                  = "/range_front";  
    std::string control_speed_line_topic           = "/control_speed/line";
    std::string control_speed_yaw_topic            = "/control_speed/yaw";
    std::string control_speed_sonar_topic          = "/control_speed/sonar";
    

    ros::param::get("~type_cmd_vel_topic",             type_cmd_vel_topic);  
    ros::param::get("~front_sonar_topic",             front_sonar_topic);
    ros::param::get("~line_centroid_topic",          line_centroid_topic);
    ros::param::get("~Kp_line_topic",                Kp_line_topic);
    ros::param::get("~target_yaw_degree_topic",       target_yaw_degree_topic);
    ros::param::get("~yaw_degree_topic",            yaw_degree_topic);
    ros::param::get("~control_speed_line_topic",       control_speed_line_topic);
    ros::param::get("~control_speed_yaw_topic",       control_speed_yaw_topic);
    ros::param::get("~control_speed_sonar_topic",       control_speed_sonar_topic);

    
    
    ros::Subscriber sub_front_sonar            = nh.subscribe(front_sonar_topic, 1, Front_Sonar_Callback);
    ros::Subscriber sub_line_centroid          = nh.subscribe(line_centroid_topic, 1, line_centroid_Callback);
    ros::Subscriber sub_yaw_degree             = nh.subscribe(yaw_degree_topic, 1, yaw_degree_Callback);
    
    
    ros::Publisher pub_type_cmd_vel           = nh.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);
    ros::Publisher pub_target_yaw_degree      = nh.advertise<std_msgs::Float64>(target_yaw_degree_topic, 1);
    ros::Publisher pub_Kp_line                = nh.advertise<std_msgs::Float64>(Kp_line_topic, 1);
    ros::Publisher pub_control_speed_line     = nh.advertise<std_msgs::Float64>(control_speed_line_topic, 1);
    ros::Publisher pub_control_speed_yaw      = nh.advertise<std_msgs::Float64>(control_speed_yaw_topic, 1);
    ros::Publisher pub_control_speed_sonar    = nh.advertise<std_msgs::Float64>(control_speed_sonar_topic, 1);


    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
      std_msgs::Int8 type_cmd_vel;
      std_msgs::Float64 target_yaw_degree;
      std_msgs::Float64 Kp_line;
      std_msgs::Float64 control_speed_line;
      std_msgs::Float64 control_speed_yaw;
      std_msgs::Float64 control_speed_sonar;
      
      switch(mission_flag)
      {
         
         case 0:
            if (find_line_center == 0.0)
            {
             type_cmd_vel.data = 0;
             control_speed_line.data = 0.0;
            }
            else
            {
                mission_flag++;
            }
            break;
            
         case 1:
            if (find_line_center != 0.0)
            {
             Kp_line.data = 0.0015;
             type_cmd_vel.data = 0;
             control_speed_line.data = 0.3;
            }
            
            if (find_line_center == 0.0)
            {
             mission_flag++;
            }
            break;
            
         case 2:
            if (front_sonar > 1.2)
            {
             type_cmd_vel.data = 1;
             target_yaw_degree.data = 0.0;
             control_speed_yaw.data = 0.3;
            }
            
            else
            {
             mission_flag++;
            }
            break;

         case 3:
            target_yaw_degree.data = 270.0;
            type_cmd_vel.data = 1;
            control_speed_yaw.data = 0.5;
         
           if(yaw_degree < 270.5 && yaw_degree >= 270)
           {
             mission_flag++;
           }
           break;

        case 4:
           type_cmd_vel.data = 2;
           control_speed_sonar.data = 0.4;
           
             if (front_sonar < 1.0)
             {
              mission_flag++;
             }
             break;
         
         case 5:
            target_yaw_degree.data = 180.0;
            type_cmd_vel.data = 1;
            control_speed_yaw.data = 0.3;
            
            if(yaw_degree < 180.5 && yaw_degree >= 179.5)         
            {
             mission_flag++;
            }
          break;

         case 6:
            if(find_line_center != 0)
            {
             Kp_line.data = 0.0018;
             type_cmd_vel.data = 0;
             control_speed_line.data = 0.3;   
            }

            else
            {
             type_cmd_vel.data = 0;
             control_speed_line.data = 0.0;         
            }
            break;
      }   
      
      printf("mission_flag = %d\n",mission_flag);
      printf("line_center = %.2lf\n",find_line_center);
      printf("front_sonar = %.2lf\n",front_sonar);
      printf("yaw_degree = %.2lf\n",yaw_degree);

      printf("target_yaw_degree = %.2lf\n\n",target_yaw_degree.data);
      
      pub_type_cmd_vel.publish(type_cmd_vel);
      pub_control_speed_line.publish(control_speed_line);
      pub_control_speed_yaw.publish(control_speed_yaw);
      pub_control_speed_sonar.publish(control_speed_sonar);
      pub_target_yaw_degree.publish(target_yaw_degree);
      pub_Kp_line.publish(Kp_line);
        
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }

    return 0;
}
