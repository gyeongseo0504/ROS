#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#define WINDOW_SIZE 3 

class MovingAverageFilter
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    MovingAverageFilter() 
    {
        sub = nh.subscribe("input_data", 10, &MovingAverageFilter::dataCallback, this);
        pub = nh.advertise<std_msgs::Int32MultiArray>("filtered_data", 10);
    }

    void dataCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) 
    {
        const std::vector<int32_t>& data = msg->data;
        int dataSize = data.size();
        
        std::vector<int32_t> filteredData(dataSize - WINDOW_SIZE + 1);

        for (int i = 0; i <= dataSize - WINDOW_SIZE; ++i) 
        {
            int sum = 0;
            for (int j = i; j < i + WINDOW_SIZE; ++j) 
            {
                sum += data[j];
            }
            filteredData[i] = sum / WINDOW_SIZE;
        }

        std_msgs::Int32MultiArray filteredMsg;
        filteredMsg.data = filteredData;
        pub.publish(filteredMsg);
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "moving_average_filter");
    
    MovingAverageFilter filter;

    ros::spin();

    return 0;
}
