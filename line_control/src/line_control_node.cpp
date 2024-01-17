#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.01  
#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

float Kp = 0.0015;
float Ki = 0.0;
float Kd = 0.005;

float Line_Center = TSL1401CL_SIZE / 2;  // Adjust this according to your sensor configuration
float OFFSET = 0;

float error = 0.0;
float error_d = 0.0;
float error_sum = 0.0;
float error_old = 0.0;  // Add this line
float Steering_Angle = 0.0;

void threshold(double tsl1401cl_data[], int ThresholdData[], int tsl1401cl_size, double threshold)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
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

void tsl1401cl_Callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }
    threshold(tsl1401cl_data, LineSensor_threshold_Data, TSL1401CL_SIZE, THRESHOLD);
}

int find_line_center()
{
    int centroid = 0;
    int mass_sum = 0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    centroid = (mass_sum != 0) ? centroid / mass_sum : 0;

    return centroid;
}

void PID_lane_control(geometry_msgs::Twist &cmd_vel)
{
    double lineCenter = find_line_center();
    
    error = Line_Center - lineCenter + OFFSET;
    error_sum += error;
    error_d = error - error_old;
    Steering_Angle = Kp * error + Ki * error_sum + Kd * error_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = Steering_Angle;
    
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
        return;
    }
    error_old = error; 
}

int main(int argc, char **argv)
{
    int count = 0;

    geometry_msgs::Twist cmd_vel;

    ros::init(argc, argv, "line_control");
    ros::NodeHandle nh;

    ros::Subscriber tsl1401cl_sub = nh.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);
    ros::Publisher tst1401cl_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);
    while (ros::ok())
    {
        printf("Threshold Data: \n");
        
        for (int i = 0; i < TSL1401CL_SIZE; i++)
        {
            printf("%d ", LineSensor_threshold_Data[i]);
        }
        printf("\n");

        double centroid = find_line_center();
        printf("Line Centroid: %f\n", centroid);

        PID_lane_control(cmd_vel);

        tst1401cl_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
