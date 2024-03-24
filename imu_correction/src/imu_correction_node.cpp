#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath> // for math functions like M_PI

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;

std_msgs::Float32 yaw_deg;
std_msgs::Float32 yaw_offset_degree;

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

void yaw_offset_degree_Callback(const std_msgs::Float32::ConstPtr &msg)
{
    yaw_offset_degree = *msg;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "imu_correction");
    ros::NodeHandle n;

    std::string imu_topic = "/imu";
    std::string yaw_offset_degree_topic = "/yaw_offset_degree";
    std::string yaw_d_corrected_topic = "/yaw_d_corrected";

    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~yaw_offset_degree_topic", yaw_offset_degree_topic);

    ros::Subscriber sub_imu = n.subscribe(imu_topic, 1, imu1Callback);
    ros::Subscriber sub_control_speed_yaw_ = n.subscribe(yaw_offset_degree_topic, 1, yaw_offset_degree_Callback);

    ros::Publisher pub_yaw_d_corrected = n.advertise<std_msgs::Float32>(yaw_d_corrected_topic, 1);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        std_msgs::Float32 yaw_d;
        yaw_d.data = yaw_deg.data + yaw_offset_degree.data;
        printf("yaw = %.2f\n", yaw_deg.data);
        printf("yaw_d = %.2f\n", yaw_d.data);

        pub_yaw_d_corrected.publish(yaw_d);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
