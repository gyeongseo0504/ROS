#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x

double roll, pitch, yaw;

void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);      

    m.getRPY(roll, pitch, yaw);
    
    double yaw_degree = yaw * 180.0 / M_PI; // Pitch, Roll, Yaw 중 yaw는 헤딩 앵글 좌우를 움직여 쓴다
    
    if (yaw_degree > 360) // 0도~ 360도 값을 갖도록 함
    {
        yaw_degree = yaw_degree - 360; // 360보다 크면 360을 빼서 360이하로 값을 나오게 함
    }
    else if (yaw_degree < 0)
    {
        yaw_degree = yaw_degree + 360;
    }

    ROS_INFO("Yaw Degree: %.2f", yaw_degree);
}
	void PID_control(geometry_msgs::Twist &cmd_vel)
{
	double Kp = 0.5
	double Ki = 0.01
	double Kd = 0.25
	
	
	double yaw_heading_degree = 0.0;
	double error_old = 0.0;
	
	double error = yaw_heading_degree - yaw_degree;//원하는 각도 와 현재 각도의 오차값
	double error_d = error - error_old;// 현재 오차와 이전 오차를 비교하여 현재 오차의 변화율을 계산
    double error_sum = error + error_old; //누적 오차를 나타내는 적분 값
    
    double steering_angle = Kp * error + Ki * error_sum + Kd * error_d;//pid 값
	
	geometry_msgs::Twist cmd_vel;
    cmd_vel.angular.z = steering_angle;
	
	return cmd_vel;
}
int main(int argc, char **argv)
{
    int count = 0;
	
    ros::init(argc, argv, "yaw_control");
    ros::NodeHandle n;
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Rate loop_rate(20.0);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
