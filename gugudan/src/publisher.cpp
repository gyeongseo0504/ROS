#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gugudan_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(1); 

    ros::Publisher pub = n.advertise<std_msgs::Int32>("answer", 10);

    while (ros::ok())
    {
        for (int i = 1; i <= 9; ++i)
        {
            for (int j = 1; j <= 9; ++j)
            {
                std_msgs::Int32 result;
                result.data = i * j;
                ROS_INFO("%d * %d", i, j);
                pub.publish(result);
                loop_rate.sleep();
            }
        }
    }

    return 0;
}
