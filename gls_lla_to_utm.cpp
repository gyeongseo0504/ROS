#include <ros/ros.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

#define PI 3.14159265358979323846
#define a 6378137.0
#define e 0.081819191
#define k0 0.9996

double latitude, longitude, altitude;
double utm_east, utm_north;
bool new_gps_data = false;

ros::Publisher utm_pub;
ros::Publisher lla_pub;

void gps_fix_lla(const sensor_msgs::NavSatFix& gps_fix, double* latitude, double* longitude) 
{
    *latitude = gps_fix.latitude;
    *longitude = gps_fix.longitude;

    printf("Received GPS data: Latitude = %9.6lf, Longitude = %9.6lf\n", *latitude, *longitude);
}

void gps1_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    gps_fix_lla(*msg, &latitude, &longitude);
    new_gps_data = true;
}

void ToUTM(double latitude, double longitude, double& utmX, double& utmY) 
{
    double lat = latitude * PI / 180.0;
    double lon = longitude * PI / 180.0;

    int zone = (int)((longitude + 180) / 6) + 1;
    double lon0 = (zone * 6 - 183) * PI / 180.0;

    double N = a / sqrt(1 - e * e * sin(lat) * sin(lat));
    double T = tan(lat) * tan(lat);
    double C = e * e * cos(lat) * cos(lat) / (1 - e * e);
    double A = (lon - lon0) * cos(lat);

    double M = a * ((1 - e * e / 4 - 3 * e * e * e * e / 64 - 5 * e * e * e * e * e * e / 256) * lat
              - (3 * e * e / 8 + 3 * e * e * e * e / 32 + 45 * e * e * e * e * e * e / 1024) * sin(2 * lat)
              + (15 * e * e * e * e / 256 + 45 * e * e * e * e * e * e / 1024) * sin(4 * lat)
              - (35 * e * e * e * e * e * e / 3072) * sin(6 * lat));

    utmX = k0 * N * (A + (1 - T + C) * A * A * A / 6
           + (5 - 18 * T + T * T + 72 * C - 58 * e * e) * A * A * A * A * A / 120)
           + 500000.0;

    utmY = k0 * (M + N * tan(lat) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
           + (61 - 58 * T + T * T + 600 * C - 330 * e * e) * A * A * A * A * A * A / 720));

    if (latitude < 0) 
    {
        utmY += 10000000.0;
    }
}

void pub_lla(double latitude, double longitude) 
{
    geometry_msgs::Pose2D lla_msg;
    lla_msg.x = latitude;
    lla_msg.y = longitude;

    lla_pub.publish(lla_msg);
    printf("Published LLA: Latitude = %9.6lf, Longitude = %9.6lf\n", latitude, longitude);
}

void pub_utm(double utmX, double utmY) 
{
    geometry_msgs::Pose2D utm_msg;
    utm_msg.x = utmX;
    utm_msg.y = utmY;

    utm_pub.publish(utm_msg);
    printf("Published UTM: utm_East = %9.6lf, utm_North = %9.6lf\n", utmX, utmY);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_to_utm_node");
    ros::NodeHandle n;

    std::string gps1_fix_topic = "/gps1/fix";
    std::string utm_pub_topic = "/utm_coordinates";
    std::string lla_pub_topic = "/lla_coordinates";
    
    ros::param::get("~gps1_fix_topic", gps1_fix_topic);
    ros::param::get("~utm_pub_topic", utm_pub_topic);
    ros::param::get("~lla_pub_topic", lla_pub_topic);

    utm_pub = n.advertise<geometry_msgs::Pose2D>(utm_pub_topic, 10);
    lla_pub = n.advertise<geometry_msgs::Pose2D>(lla_pub_topic, 10);

    ros::Subscriber sub_gps1_fix = n.subscribe(gps1_fix_topic, 1, gps1_Callback);

    ros::Rate loop_rate(25.0);

    while (ros::ok())
    {
        if (new_gps_data)
        {
            ToUTM(latitude, longitude, utm_east, utm_north);
            printf("Converted to UTM : utm_East = %9.6lf utm_North = %9.6lf\n\n", utm_east, utm_north);
            
            pub_lla(latitude, longitude);
            pub_utm(utm_east, utm_north);

            new_gps_data = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
