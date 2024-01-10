#include "ros/ros.h"
#include "std_msgs/Int32.h"

void gugudanCallback(const std_msgs::Int32::ConstPtr &data)
{
    ROS_INFO("answer: %d", data->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gugudan_subscriber_node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("answer", 10, gugudanCallback);

    ros::spin();

    return 0;
}
