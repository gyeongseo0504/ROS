#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"

#define WALL_GAP_DISTANCE_HALP 1.0

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;
double error_old = 0.0;
int maze_status = 0;

void sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  if (msg->header.frame_id == "front_sonar")
  {
    front_sonar = msg->range;
    ROS_INFO("Front_Sonar Range: [%f] meters", front_sonar);
  }
  else if (msg->header.frame_id == "left_sonar")
  {
    left_sonar = msg->range;
    ROS_INFO("Left_Sonar Range: [%f] meters", left_sonar);
  }
  else if (msg->header.frame_id == "right_sonar")
  {
    right_sonar = msg->range;
    ROS_INFO("Right_Sonar Range: [%f] meters", right_sonar);
  }
  else
  {
    ROS_WARN("Unknown sonar frame_id: %s", msg->header.frame_id.c_str());
  }
}

void wall_following(const double &pid, geometry_msgs::Twist &cmd_vel)
{
  double Kp = 0.8;
  double Ki = 0.0;
  double Kd = 0.8;
  
  double error = left_sonar - right_sonar;
  double error_d = error - error_old;  
  double error_sum = 0.0;
  error_sum += error;  
  
  double steering_control = Kp * error + Ki * error_sum + Kd * error_d;
    
  if (front_sonar < 1.2)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  else
  {
    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = steering_control + pid; 
  }

  error_old = error; 
}

int main(int argc, char **argv)
{
  int count = 0;
  
  geometry_msgs::Twist cmd_vel;
  
  ros::init(argc, argv, "wall_following");
  ros::NodeHandle n;
  
  ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, sonarCallback);
  ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, sonarCallback);
  ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, sonarCallback);
  
  ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

  ros::Rate loop_rate(30.0);  
  
  while (ros::ok())
  {
    double pid_value = 0.0; 
    wall_following(pid_value, cmd_vel);
    sonar_cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
