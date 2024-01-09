#include "ros/ros.h"
#include "std_msgs/Int32.h"

void gugudanCallback(const std_msgs::Int32::ConstPtr &msg)
{
    ROS_INFO("Received: %d", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gugudan_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber gugudan_sub = nh.subscribe("gugudan_results", 10, gugudanCallback);

    ros::spin();

    return 0;
}
