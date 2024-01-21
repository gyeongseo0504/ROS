#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

#define TSL1401CL_SIZE 320
#define THRESHOLD 0.1
#define Line_Center 160.0
#define OFFSET -13.0

double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];
int mission_flag = 0;

double line_error_old = 0.0;

double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;
double error_old = 0.0;
int maze_status = 0;

double roll, pitch, yaw;
double error_wall_old = 0.0;
double error_yaw_old = 0.0;
double error_lane_old = 0.0;

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

double find_line_center()
{
    double centroid = 0.0;
    double mass_sum = 0.0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum != 0)
    {
        centroid = centroid / mass_sum;
    }

    return centroid;
}

geometry_msgs::Twist PID_lane_control(double Kp_lane, double Ki_lane, double Kd_lane)
{
    geometry_msgs::Twist cmd_vel;

    double lineCenter = find_line_center();

    double error_lane = Line_Center - lineCenter + OFFSET;
    double error_lane_d = error_lane - error_lane_old;
    double error_lane_sum = 0.0;

    error_lane_sum += error_lane;

    double steering_angle = Kp_lane * error_lane + Ki_lane * error_lane_sum + Kd_lane * error_lane_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = steering_angle;

    error_lane_old = error_lane;

    return cmd_vel;
}

geometry_msgs::Twist PID_wall_following(double Kp_wall, double Ki_wall, double Kd_wall)
{
    geometry_msgs::Twist cmd_vel;

    double error_wall = left_sonar - right_sonar;
    double error_wall_d = error_wall - error_wall_old;
    double error_wall_sum = 0.0;
    error_wall_sum += error_wall;

    double steering_control = Kp_wall * error_wall + Ki_wall * error_wall_sum + Kd_wall * error_wall_d;

    if (front_sonar < 0.9)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        cmd_vel.linear.x = 0.6;
        cmd_vel.angular.z = steering_control;
    }

    error_wall_old = error_wall;

    return cmd_vel;
}

geometry_msgs::Twist PID_yaw_control(double Kp_yaw, double Ki_yaw, double Kd_yaw, double target_yaw_degree)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_degree = yaw * 180.0 / M_PI;

    if (yaw_degree > 360)
    {
        yaw_degree = yaw_degree - 360;
    }
    else if (yaw_degree < 0)
    {
        yaw_degree = yaw_degree + 360;
    }

    double error_yaw = target_yaw_degree - yaw_degree;

    if (error_yaw > 180)
    {
        error_yaw = error_yaw - 360;
    }
    else if (error_yaw < -180)
    {
        error_yaw = error_yaw + 360;
    }

    double error_yaw_sum = 0.0;
    double error_yaw_d = error_yaw - error_yaw_old;

    error_yaw_sum += error_yaw;

    double Steering_Angle = Kp_yaw * error_yaw + Ki_yaw * error_yaw_sum + Kd_yaw * error_yaw_d;

    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(error_yaw) < 1.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        mission_flag++;
    }

    error_yaw_old = error_yaw;

    return cmd_vel;
}

void Front_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    front_sonar = msg->range;
}

void Left_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    left_sonar = msg->range;
}

void Right_Sonar_Callback(const sensor_msgs::Range::ConstPtr &msg)
{
    right_sonar = msg->range;
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

    double yaw_degree = yaw * 180.0 / M_PI;

    if (yaw_degree > 360)
    {
        yaw_degree = yaw_degree - 360;
    }
    else if (yaw_degree < 0)
    {
        yaw_degree = yaw_degree + 360;
    }
}

int main(int argc, char **argv)
{
    int count = 0;
    double target_yaw_degree = 0.0;

    double Kp_lane = 0.0015;
    double Ki_lane = 0.0;
    double Kd_lane = 0.005;

    double Kp_wall = 1.2;
    double Ki_wall = 0.0;
    double Kd_wall = 0.8;

    double Kp_yaw = 0.02;
    double Ki_yaw = 0.0;
    double Kd_yaw = 0.5;

    ros::init(argc, argv, "pioneer");
    ros::NodeHandle n;

    geometry_msgs::Twist cmd_vel;

    ros::Subscriber front_sonar_sub = n.subscribe("/range_front", 1000, Front_Sonar_Callback);
    ros::Subscriber left_sonar_sub = n.subscribe("/range_front_left", 1000, Left_Sonar_Callback);
    ros::Subscriber right_sonar_sub = n.subscribe("/range_front_right", 1000, Right_Sonar_Callback);
    ros::Subscriber imu_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Subscriber tsl1401cl_sub = n.subscribe("/tsl1401cl", 10, tsl1401cl_Callback);

    ros::Publisher sonar_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        switch (mission_flag)
        {
        case 0:
            if (find_line_center() == -1.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                mission_flag++;
            }
            break;

        case 1:
            if (find_line_center() != 0.0)
            {
                cmd_vel = PID_lane_control(Kp_lane, Ki_lane, Kd_lane);
            }
            if (find_line_center() == 0.0)
            {
                mission_flag++;
            }
            break;

        case 2:
            if (front_sonar > 1.1)
            {
                cmd_vel.linear.x = 0.9;
                cmd_vel.angular.z = 0.0;
            }
            else
            {
                target_yaw_degree = 270.0;
                mission_flag++;
            }
            break;

        case 3:
            cmd_vel = PID_yaw_control(Kp_yaw, Ki_yaw, Kd_yaw, target_yaw_degree);
            break;

        case 4:
            cmd_vel = PID_wall_following(Kp_wall, Ki_wall, Kd_wall);
            if (front_sonar < 1.1)
            {
                target_yaw_degree = 180.0;
                mission_flag++;
            }
            break;

        case 5:
            cmd_vel = PID_yaw_control(Kp_yaw, Ki_yaw, Kd_yaw, target_yaw_degree);
            break;

        case 6:
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
                cmd_vel = PID_lane_control(Kp_lane, Ki_lane, Kd_lane);
            }
            break;
        }

        sonar_cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
