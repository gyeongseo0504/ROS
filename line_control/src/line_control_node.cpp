#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include <std_msgs/Float32MultiArray.h>
#include <math.h>

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

///////////// CAMERA /////////////
#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01
#define Line_Center 147
#define OFFSET 13

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

double Kp_line;
double Ki_line          = 0.0;
double Kd_line          = 0.0;
double Kp_curve_line    = 0.0;
double control_speed_line;

std_msgs::Float64 centroid;

void control_speed_line_Callback(const std_msgs::Float64::ConstPtr &msg)
{
   control_speed_line = msg->data;
}

void Kp_line_Callback(const std_msgs::Float64::ConstPtr &msg)
{
   Kp_line = msg->data;
}

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < tsl1401cl_size; i++)
    {
        if (tsl1401cl_data[i] > threshold)
        {
            ThresholdData[i] = 255;
        }
        else
        {
            ThresholdData[i] = 0;
        }
    }
}

double find_line_center()
{
    double mass_sum = 0.0;
    centroid.data = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid.data += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum == 0)
    {
        mass_sum = 1.0;
    }

    centroid.data = centroid.data/ mass_sum;

    return centroid.data;
}

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
    
    printf("Threshold Data: \n");

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");

    double centroid = find_line_center();
    printf("Line Centroid: %f\n", centroid);
}

double error_lane_old = 0.0;

geometry_msgs::Twist PID_lane_control(double Kp_line, double Ki_line, double Kd_line)
{
    geometry_msgs::Twist cmd_vel;

    double lineCenter = find_line_center();

    double error_lane = Line_Center - lineCenter + OFFSET;
    double error_lane_d = error_lane - error_lane_old;
    double error_lane_sum = 0.0;

    error_lane_sum += error_lane;

    double steering_angle = Kp_line * error_lane + Ki_line * error_lane_sum + Kd_line * error_lane_d;

    cmd_vel.linear.x = control_speed_line;
    cmd_vel.angular.z = steering_angle;
/*
///////////// black ///////////// 
    bool recognize_X = true;
    
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (LineSensor_threshold_Data[i] != 0)
        {
            recognize_X = false;
            break;
        }
    }

    if (recognize_X)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        return cmd_vel;
    }
/*    
///////////// white ///////////// 
    bool finish = true;
    
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (LineSensor_threshold_Data[i] != 255)
        {
         finish = false;
         break;
      }
    }

    if (finish)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel = PID_lane_control(Kp_curve_line, Ki_line, Kd_line);
    }
*/
    error_lane_old = error_lane;

    return cmd_vel;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;
    
    std::string line_topic                   = "/tsl1401cl";
    std::string line_cmd_vel_topic             = "/cmd_vel/line";
    std::string line_centroid_topic            = "/line_centroid";
    std::string control_speed_line_topic      = "/control_speed/line";
    std::string Kp_line_topic               = "/Kp_line";
    
    ros::param::get("~line_topic",line_topic);
    ros::param::get("~line_cmd_vel_topic",line_cmd_vel_topic);
    ros::param::get("~line_centroid_topic",  line_centroid_topic);
   ros::param::get("~control_speed_line_topic", control_speed_line_topic);

    ros::param::get("~Kp_line_topic", Kp_line_topic);  
   ros::param::get("~Ki_line", Ki_line);  
   ros::param::get("~Kd_line", Kd_line);
   
   ros::Subscriber sub_control_speed_line = nh.subscribe(control_speed_line_topic, 1, control_speed_line_Callback);
   ros::Subscriber sub_Kp_line = nh.subscribe(Kp_line_topic, 1, Kp_line_Callback);
   ros::Subscriber line_control_sub = nh.subscribe(line_topic, 1, tsl1401cl_Callback);
    ros::Publisher line_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(line_cmd_vel_topic, 1);
    ros::Publisher pub_line_centroid = nh.advertise<std_msgs::Float64>(line_centroid_topic, 1);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        geometry_msgs::Twist cmd_vel_line = PID_lane_control(Kp_line, Ki_line, Kd_line);
     
        line_cmd_vel_pub.publish(cmd_vel_line);
        pub_line_centroid.publish(centroid);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
