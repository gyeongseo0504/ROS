#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;
double error_old = 0.0;
double target_yaw_degree;
double control_speed_yaw;

double Kp_yaw = 0.0;
double Ki_yaw = 0.0;
double Kd_yaw = 0.0;

double error;
double yaw_d_tf_correction = 90;

std_msgs::Float64 yaw_deg;

double normalizeYaw(double yaw_deg)
{
    if (yaw_deg > 360)
    {
        yaw_deg = yaw_deg - 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;
    }

    return yaw_deg;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg_value = normalizeYaw(RAD2DEG(yaw));

    yaw_deg.data = yaw_deg_value;
}


void target_yaw_degree_Callback(const std_msgs::Float64::ConstPtr &msg)
{
   target_yaw_degree = msg->data;
}

void control_speed_yaw_Callback(const std_msgs::Float64::ConstPtr &msg)
{
   control_speed_yaw = msg->data;
}

geometry_msgs::Twist PID_yaw_control(double Kp_yaw, double Ki_yaw, double Kd_yaw)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = normalizeYaw(yaw_deg);

    error = target_yaw_degree + yaw_d_tf_correction - yaw_deg;
    //error = target_yaw_degree - yaw_deg;

    if (error > 180)
    {
        error = error - 360;
    }
    else if (error < -180)
    {
        error = error + 360;
    }

    double error_sum = 0.0;
    double error_d = error - error_old;

    error_sum += error;

    double Steering_Angle = Kp_yaw * error + Ki_yaw * error_sum + Kd_yaw * error_d;

    cmd_vel.linear.x = control_speed_yaw;
    cmd_vel.angular.z = Steering_Angle;
/*
    if (fabs(error) < 0.5)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
*/   
    error_old = error;

    return cmd_vel;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    
    std::string imu_topic                = "/imu";
    std::string target_yaw_degree_topic    = "/target_yaw/degree";
    std::string yaw_cmd_vel_topic          = "/cmd_vel/yaw";
    std::string control_speed_yaw_topic    = "/control_speed/yaw";
    std::string yaw_degree_topic          = "/yaw_degree";
    
   ros::param::get("~imu_topic",               imu_topic);
   ros::param::get("~target_yaw_degree_topic",      target_yaw_degree_topic);
    ros::param::get("~yaw_cmd_vel_topic",         yaw_cmd_vel_topic);
    ros::param::get("~control_speed_yaw_topic",      control_speed_yaw_topic);
   ros::param::get("~Kp_yaw",                   Kp_yaw);  
   ros::param::get("~Kd_yaw",                   Kd_yaw);  
   ros::param::get("~Ki_yaw",                   Ki_yaw);
   ros::param::get("~yaw_d_tf_correction",       yaw_d_tf_correction);
    ros::param::get("~target_yaw_degree",          target_yaw_degree);
    ros::param::get("~yaw_degree_topic",          yaw_degree_topic);

    ros::Subscriber sub_target_yaw_degree   = n.subscribe(target_yaw_degree_topic, 1, target_yaw_degree_Callback);
    ros::Subscriber sub_imu                = n.subscribe(imu_topic, 1, imu1Callback);
    ros::Subscriber sub_control_speed_yaw_    = n.subscribe(control_speed_yaw_topic, 1, control_speed_yaw_Callback);
    ros::Publisher pub_yaw_cmd_vel          = n.advertise<geometry_msgs::Twist>(yaw_cmd_vel_topic, 1);
    ros::Publisher pub_yaw_degree          = n.advertise<std_msgs::Float64>(yaw_degree_topic, 1);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {

        geometry_msgs::Twist cmd_vel_yaw = PID_yaw_control(Kp_yaw, Ki_yaw, Kd_yaw);
            
      printf("yaw = %.2f\n", yaw_deg.data);
      printf("target_yaw_degree = %.2lf\n", target_yaw_degree);
      printf("error = %.2lf\n",error);
      printf("speed = %.2lf\n\n",control_speed_yaw);
        
        pub_yaw_cmd_vel.publish(cmd_vel_yaw);
        pub_yaw_degree.publish(yaw_deg);
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
