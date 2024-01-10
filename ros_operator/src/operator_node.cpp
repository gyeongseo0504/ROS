#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

float x_value = 0.0;
std::string operator_value = "";
float y_value = 0.0;

void xCallback(const std_msgs::Float32::ConstPtr& msg)
{
    x_value = msg->data;
}

void operatorCallback(const std_msgs::String::ConstPtr& msg)
{
    operator_value = msg->data;
}

void yCallback(const std_msgs::Float32::ConstPtr& msg)
{
    y_value = msg->data;

    double result = 0.0;

    if (operator_value == "+") 
    {
        result = x_value + y_value;
    } 
    else if (operator_value == "-")
    {
        result = x_value - y_value;
    } 
    else if (operator_value == "x")
    {
        result = x_value * y_value;
    } 
    else if (operator_value == "/") 
    {
        if (y_value != 0.0) 
        {
            result = x_value / y_value;
        } else 
        {
            ROS_WARN("Division by zero is not allowed.");
            return;
        }
    } 
    else 
    {
        ROS_WARN_STREAM("Unsupported operator: " << operator_value);
        return;
    }

    printf("%1.0f %s %1.0f = %1.1f\n", x_value, operator_value.c_str(), y_value, result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber x_sub = nh.subscribe("/float32/x", 1, xCallback);
    ros::Subscriber operator_sub = nh.subscribe("/string/operator", 1, operatorCallback);
    ros::Subscriber y_sub = nh.subscribe("/float32/y", 1, yCallback);

    ros::spin();

    return 0;
}
