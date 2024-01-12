#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

double x_value = 0.0;
std::string operator_value = "";
double y_value = 0.0;

bool a_received = false;
bool b_received = false;

ros::Publisher result_pub;

void xCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_value = msg->data;
    a_received = true;
}

void operatorCallback(const std_msgs::String::ConstPtr& msg)
{
    operator_value = msg->data;
}

void yCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_value = msg->data;
    b_received = true;

    if (a_received && b_received) {
        double result = 0.0;

        if (operator_value == "+") {
            result = x_value + y_value;
        } else if (operator_value == "-") {
            result = x_value - y_value;
        } else if (operator_value == "x") {
            result = x_value * y_value;
        } else if (operator_value == "/") {
            if (y_value != 0.0) {
                result = x_value / y_value;
            } else {
                ROS_WARN("WARNING: Division by zero");
                return;
            }
        } else {
            ROS_WARN("WARNING: Invalid operator");
            return;
        }

        ROS_INFO_STREAM(x_value << " " << operator_value << " " << y_value << " = " << result);

        std_msgs::Float32 result_msg;
        result_msg.data = result;
        result_pub.publish(result_msg);

        x_value = 0.0;
        operator_value = "";
        y_value = 0.0;

        a_received = false;
        b_received = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculator_node");
    ros::NodeHandle nh;

    ros::Publisher x_pub = nh.advertise<std_msgs::Float32>("/float32/x", 1);
    ros::Publisher operator_pub = nh.advertise<std_msgs::String>("/string/operator", 1);
    ros::Publisher y_pub = nh.advertise<std_msgs::Float32>("/float32/y", 1);

    result_pub = nh.advertise<std_msgs::Float32>("/result", 1);

    ros::Subscriber x_sub = nh.subscribe("/float32/x", 1, xCallback);
    ros::Subscriber operator_sub = nh.subscribe("/string/operator", 1, operatorCallback);
    ros::Subscriber y_sub = nh.subscribe("/float32/y", 1, yCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
