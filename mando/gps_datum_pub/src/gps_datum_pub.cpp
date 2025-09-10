#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"

#include "geometry_msgs/Vector3.h"
#include <XmlRpcException.h>

bool use_gps_init_datum = false;
int no_waypoints = 0;
struct WayPoints
{
	double x;
	double y;	
	double theta;
};

struct WayPoints my_waypoints_list[1];


// Datum parameter - required
void ToLatLon(double utmX, double utmY, const char* utmZone, double* latitude, double* longitude)
{
    int isNorthHemisphere = utmZone[strlen(utmZone) - 1] >= 'N';

    double diflat = -0.00066286966871111111111111111111111111;
    double diflon = -0.0003868060578;

    int zone;
    sscanf(utmZone, "%d", &zone);
    double c_sa = 6378137.000000;
    double c_sb = 6356752.314245;
    double e2 = sqrt((pow(c_sa, 2) - pow(c_sb, 2))) / c_sb;
    double e2cuadrada = pow(e2, 2);
    double c = pow(c_sa, 2) / c_sb;
    double x = utmX - 500000;
    double y = isNorthHemisphere ? utmY : utmY - 10000000;

    double s = ((zone * 6.0) - 183.0);
    double lat = y / (c_sa * 0.9996);
    double v = (c / sqrt(1 + (e2cuadrada * pow(cos(lat), 2)))) * 0.9996;
    double a = x / v;
    double a1 = sin(2 * lat);
    double a2 = a1 * pow((cos(lat)), 2);
    double j2 = lat + (a1 / 2.0);
    double j4 = ((3 * j2) + a2) / 4.0;
    double j6 = ((5 * j4) + pow(a2 * (cos(lat)), 2)) / 3.0;
    double alfa = (3.0 / 4.0) * e2cuadrada;
    double beta = (5.0 / 3.0) * pow(alfa, 2);
    double gama = (35.0 / 27.0) * pow(alfa, 3);
    double bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
    double b = (y - bm) / v;
    double epsi = ((e2cuadrada * pow(a, 2)) / 2.0) * pow((cos(lat)), 2);
    double eps = a * (1 - (epsi / 3.0));
    double nab = (b * (1 - epsi)) + lat;
    double senoheps = (exp(eps) - exp(-eps)) / 2.0;
    double delt = atan(senoheps / (cos(nab)));
    double tao = atan(cos(delt) * tan(nab));

    *longitude = ((delt * (180.0 / M_PI)) + s) + diflon;
    *latitude = ((lat + (1 + e2cuadrada * pow(cos(lat), 2) - (3.0 / 2.0) * e2cuadrada * sin(lat) * cos(lat) * (tao - lat)) * (tao - lat)) * (180.0 / M_PI)) + diflat;
}


void init_waypoint(const std::string& file_path)
{
	FILE *fp;

	printf("waypoint file:%s\n", file_path.c_str());	
	
	fp = fopen(file_path.c_str(), "r");
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoints_data does not exit!");
		
		my_waypoints_list[0].x = 1;   
		my_waypoints_list[0].y = 1;				
		exit(1);
	}
	else
	{
	    no_waypoints = 0;
	    
		fscanf(fp,"%lf %lf  %lf",&my_waypoints_list[no_waypoints].x, &my_waypoints_list[no_waypoints].y,&my_waypoints_list[no_waypoints].theta);							   
		
		ROS_INFO("WayPoints Number %d",no_waypoints);
		for(int i=0; i<1; i++)
		{
			ROS_INFO("WayPoints-%d : [%.2lf %.2lf %.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y, my_waypoints_list[i].theta);
		}
		fclose(fp);
	}
}
int main(int argc, char **argv)
{
	std::string waypoint_file_name = "/home/kyeongseo/onefifth_catkin_ws/r.txt";
	
	double datum_lat;
	double datum_lon;
	double datum_yaw;
	double datum_utm_e, datum_utm_n;
	const char* utmZone = "52N";
	ros::init(argc, argv, "GPS_Datum_Publisher");

	ros::NodeHandle n;
	ros::NodeHandle nh_priv("~");

	ros::Publisher pub = n.advertise<geometry_msgs::Vector3>("/gps/datum", 10);
	
	printf("use_gps_init_datum : %d\n",use_gps_init_datum);
	
	if(!nh_priv.hasParam("datum") )  
	{
	  ROS_FATAL("private <datum> parameter is not supplied in "  "geonav_transform configuration");
	  exit(1);
	}

	XmlRpc::XmlRpcValue datum_config;    
	nh_priv.getParam("datum", datum_config);
	ros::param::get("~waypoint_file_name", waypoint_file_name);         //gps datum 수신 향후 처리	
	ros::param::get("~use_gps_init_datum", use_gps_init_datum);         //gps datum 수신 향후 처리
	

	ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(datum_config.size() >= 3);
      
	if(datum_config.size() > 3)
	{
		ROS_WARN_STREAM("Deprecated datum parameter configuration detected. "
						"Only the first three parameters "
						"(latitude, longitude, yaw) will be used. frame_ids "
						"will be derived from odometry and navsat inputs.");
	}
	  // Parse the data spec.
	std::ostringstream ostr;
	ostr << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
	std::istringstream istr(ostr.str());
	istr >> datum_lat >> datum_lon >> datum_yaw;
	
	init_waypoint(waypoint_file_name);
	if(use_gps_init_datum == false)
	{
		datum_utm_e = my_waypoints_list[0].x;
		datum_utm_n = my_waypoints_list[0].y;
		
		ToLatLon(my_waypoints_list[0].x, my_waypoints_list[0].y, utmZone, &datum_lat, &datum_lon);
	}
	
	ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);  // latitude , longite , yaw(north refrenece)
	ROS_INFO("UTM Datum [%12.9lf %12.9lf %12.9lf]", datum_utm_e, datum_utm_n, datum_yaw);  // latitude , longite , yaw(north refrenece)

	ros::Rate loop_rate(5);  //5
	while (ros::ok())
	{
		geometry_msgs::Vector3 GPS_Datum;
		//Clear array
		
		GPS_Datum.x = datum_lat;
		GPS_Datum.y = datum_lon;
		GPS_Datum.z = datum_yaw;
		//for loop, pushing data in the size of the array		
		
		//Publish array
		pub.publish(GPS_Datum);
		//Let the world know
		ROS_INFO("GPS Datum [%12.9lf %12.9lf %12.9lf]", datum_lat, datum_lon, datum_yaw);
		//Do this.
		loop_rate.sleep();
        ros::spinOnce();
	}

}
