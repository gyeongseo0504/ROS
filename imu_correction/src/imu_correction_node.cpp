#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;
double yaw_deg;
double yaw_offset_degree;

double normalizeYawDegree()
{
    if (yaw_deg > 360)
    {
        yaw_deg -= 360;
    }
    else if (yaw_deg < 0)
    {
        yaw_deg += 360;
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

    yaw_deg = RAD2DEG(yaw);

    double yaw_deg_value = normalizeYawDegree();

    yaw_deg = yaw_deg_value;
}

void yawOffsetDegreeCallback(const std_msgs::Float64::ConstPtr &msg)
{
    yaw_offset_degree = msg->data;
}

int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "imu_correction_node"); // Change node name
    ros::NodeHandle nh;

    std::string imu_topic = "/imu";
    std::string yaw_offset_degree_topic = "/yaw_offset_degree";
    std::string yaw_d_corrected_topic = "/yaw_d_corrected";

    ros::param::get("~imu_topic", imu_topic);
    ros::param::get("~yaw_offset_degree_topic", yaw_offset_degree_topic);
    ros::param::get("~yaw_d_corrected_topic", yaw_d_corrected_topic);

    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 1, imu1Callback);
    ros::Subscriber sub_yaw_offset_degree = nh.subscribe(yaw_offset_degree_topic, 1, yawOffsetDegreeCallback);

    ros::Publisher pub_yaw_d_corrected = nh.advertise<std_msgs::Float64>(yaw_d_corrected_topic, 1);

    ros::Rate loop_rate(30.0);

    while (ros::ok())
    {
        std_msgs::Float64 yaw_d_corrected;

        yaw_d_corrected.data = yaw_deg + yaw_offset_degree;
        printf("yaw_d_corrected = %lf\n", yaw_d_corrected.data);

        pub_yaw_d_corrected.publish(yaw_d_corrected);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
