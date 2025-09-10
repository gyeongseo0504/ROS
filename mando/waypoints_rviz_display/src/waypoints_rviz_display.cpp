//2022.08.21 gps/datum
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 500

int no_waypoints;

double datum_lat;
double datum_lon;
double datum_yaw;
double datum_utm_east;
double datum_utm_north;

double waypoint_line_angle = 0.0;

bool use_gps_init_datum = true;

int target_goal_id = 0;
bool flag_gps_datum_topic = 0;
geometry_msgs::Pose2D my_target_pose_start;
geometry_msgs::Pose2D my_target_pose_goal;

struct Point3D
{
  float x;
  float y;
  float z;
};

struct WayPoints
{
	double x;
	double y;	
};


struct WayPoints my_waypoints_list[WayPoints_NO];

void wgs2utm(double lat, double lon, int zone , double& east, double& north)
{
	double lat_rad = lat * M_PI/180;
	double lon_rad = lon * M_PI/180;

	double phi = lat_rad;
	double lambda = lon_rad;
	double lambda0 = (zone * 6 -183) * M_PI/180;
	double sm_a = 6378137;
	double sm_b = 6356752.31;

	double ep2 = (pow(sm_a, 2.0) - pow(sm_b, 2.0)) / pow(sm_b, 2.0);
	double nu2 = ep2*pow(cos(phi), 2.0);
	double N = pow(sm_a, 2.0) / (sm_b * sqrt(1 + nu2));
	double l = lambda - lambda0;
	double t = tan(phi);
	double t2 = t * t;

	double l3coef = 1 - t2 + nu2;
	double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);
	double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;
	double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;
	double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);
	double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

	east = N * cos(phi) * l + 
		(N / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) + 
		(N / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) + 
		(N / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

	double n = (sm_a - sm_b) / (sm_a + sm_b);
	double alpha = ((sm_a + sm_b) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n,4.0) / 64.0));
	double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);
	double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n,4.0) / 32.0);
	double delta = (-35.0 * pow(n,3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);
	double epsilon = (315.0 * pow(n, 4.0) / 512.0);

	double ArcLengthMeridian = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0  * phi)) + (epsilon * sin(8.0 * phi)));

	north = ArcLengthMeridian + 
			(t / 2.0 * N * pow(cos(phi), 2.0) * pow(l, 2.0)) + 
			(t / 24.0 * N * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) + 
			(t / 720.0 * N * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) + 
			(t / 40320.0 * N * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
}


void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  //printf("datum test\n");	
  datum_lat = msg->x;               
  datum_lon = msg->y;
  datum_yaw = msg->z;  
  flag_gps_datum_topic = 1;
  //ROS_INFO("GPS datum Topic Received");
  
}

void startPoseCallback(const geometry_msgs::Pose2D& msg)
{
	my_target_pose_start.y = msg.y;
	my_target_pose_start.x = msg.x;
	my_target_pose_start.theta = msg.theta;	
}

void targetPoseCallback(const geometry_msgs::Pose2D& msg)
{
	my_target_pose_goal.y = msg.y;
	my_target_pose_goal.x = msg.x;
	my_target_pose_goal.theta = msg.theta;	
}

void target_goal_id_Callback(const std_msgs::Int16::ConstPtr& msg)
{
	target_goal_id = msg->data;	
}



void init_waypoints(const std::string& file_path)
{
	FILE *fp;
	int result = -10;
	
	printf("waypoint file:%s\n", file_path.c_str());	
	
	
	fp = fopen(file_path.c_str(), "r");
	
	if(fp == NULL)
	{
		ROS_ERROR("Waypoints_data does not exit!");
		
		my_waypoints_list[0].x = 1;   
		my_waypoints_list[0].y = 2;

		my_waypoints_list[1].x = 1;   
		my_waypoints_list[1].y = 4;

		my_waypoints_list[2].x = 2;   
		my_waypoints_list[2].y = 6;  		

		my_waypoints_list[3].x = 3;   
		my_waypoints_list[3].y = 10; 
		
		no_waypoints = 4;
	}
	else
	{
		
		printf("File open success!\n");
		
		no_waypoints = -1;
		do
		{
			++no_waypoints;
			printf("%d \n", no_waypoints);
			double temp;
			result = fscanf(fp,"%lf %lf %lf",&my_waypoints_list[no_waypoints].x, &my_waypoints_list[no_waypoints].y, &temp);	
			printf("%2d %lf %lf %lf\n",no_waypoints, my_waypoints_list[no_waypoints].x, my_waypoints_list[no_waypoints].y, temp);							   
		} while(result != EOF);				
	   
	    ROS_INFO("WayPoints Number %d",no_waypoints);
	    
	    for(int i=0; i<no_waypoints; i++)
	    {
			ROS_INFO("WayPoints-%d : [%.2lf %.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y);
	    }
	    fclose(fp);	    
	    
   }
}


int main(int argc, char **argv)
{
	//std::string waypoint_file_name = "/home/amap/one_fifth_catkin_ws/src/GPS_Package/GPS_GUI/scripts/GPS_Waypoint_Ui/yongin_wp.txt2";
	std::string waypoint_file_name = "/home/kyeongseo/one_fifth_catkin_ws/waypoints/r.txt";
	
	
	Point3D p;
	std::vector<Point3D> vec_point;
	double zone = 52;

	ros::init(argc, argv, "waypoint_marker_display");
	ros::NodeHandle n;
	datum_lat = datum_lon = 0.0;
	std::vector<double> gps_init_datum;
	
	ros::param::get("~waypoint_file_name",waypoint_file_name);         //gps datum 수신 향후 처리	
	ros::param::get("~gps_init_datum", gps_init_datum);         //gps datum 수신 향후 처리
	ros::param::get("~use_gps_init_datum", use_gps_init_datum); //gps datum 수신 향후 처리

    	
	printf("GPS Init Datum: ");
	for (size_t i = 0; i < gps_init_datum.size(); ++i)
	{
		printf("%f ", gps_init_datum[i]);
	}
	printf("\n");

	init_waypoints(waypoint_file_name);

	//printf("use_gps_init_datum : %d \n\n",use_gps_init_datum);
	datum_lat = 0;               
	datum_lon = 0;
	datum_yaw = 0;
	if(use_gps_init_datum == true)
	{
	 datum_lat = gps_init_datum[0];               
	 datum_lon = gps_init_datum[1];
	 datum_yaw = gps_init_datum[2];     
	}

	ros::Subscriber sub_gps_datum      = n.subscribe("/gps/datum",1,&gps_datum_Callback);       // front gps   
	ros::Subscriber sub_target_goal_id = n.subscribe("/wp/target_id",1,&target_goal_id_Callback);  
	ros::Subscriber sub_target_goal    = n.subscribe("/wp/target_goal",10, &targetPoseCallback);

	ros::Subscriber sub_target_start   = n.subscribe("/wp/target_start",10, &startPoseCallback);


	ros::Publisher marker_pub          = n.advertise<visualization_msgs::MarkerArray>("marker/waypoint", 1);
	ros::Publisher marker_line_pub     = n.advertise<visualization_msgs::Marker>("marker/line", 1);

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	visualization_msgs::Marker path_line;


	if(use_gps_init_datum == true)   // use param datum initialization
	{
	   vec_point.clear();
		 
	   if(datum_lon < 0)
	   {
		  zone = (datum_lon + 180) / 6 + 1;
	   }
	   else
	   {
		 zone = datum_lon / 6 + 31;
	   }
	  
	   wgs2utm( datum_lat,datum_lon, zone, datum_utm_east, datum_utm_north);
	   
	   datum_utm_east = datum_utm_east * 0.9996 + 500000;
	   datum_utm_north = datum_utm_north * 0.9996;
	   
	   printf("use topic datum initialization %8.4lf %8.4lf %8.4lf %8.4lf \n\n", datum_lat, datum_lon, datum_utm_east, datum_utm_north);  

	}
	/*
	for(int i=0; i<no_waypoints; i++)
	{
			
		
		//printf("use GPS datum topic initialization\n\n");
		
		if(use_gps_init_datum == true)
		{
			p.x = my_waypoints_list[i].x - datum_utm_east;
			p.y = my_waypoints_list[i].y - datum_utm_north;
		}
		else
		{	
			//printf("dataum %d\n ", use_gps_init_datum);
	
			p.x = my_waypoints_list[i].x - my_waypoints_list[0].x;
			p.y = my_waypoints_list[i].y - my_waypoints_list[0].y;
			
		}
				
		p.z = 0.0;
			
		//printf("pos [%6.3lf %6.3lf] \n",p.x,p.y);		
	}
	
	
	*/
    if(use_gps_init_datum == true)
	{
		vec_point.clear();
		//printf("%6.3lf %6.3lf \n", datum_lat,datum_lon);
		
		if(datum_lon < 0)
		{
			zone = (datum_lon + 180) / 6 + 1;
		}
		else
		{
			zone = datum_lon / 6 + 31;
		}
		
		wgs2utm( datum_lat,datum_lon, zone, datum_utm_east, datum_utm_north);
	   
		datum_utm_east = datum_utm_east * 0.9996 + 500000;
		datum_utm_north = datum_utm_north * 0.9996;
		
		for(int i=0; i<no_waypoints; i++)
		{
			p.x = my_waypoints_list[i].x - datum_utm_east;
			p.y = my_waypoints_list[i].y - datum_utm_north;
			p.z = 0.0;
		
			//printf("1: pos [%6.3lf %6.3lf] \n",p.x,p.y);
			vec_point.push_back(p);
		}
	}
	else
	{
		vec_point.clear();
		for(int i=0; i<no_waypoints; i++)
		{
			p.x = my_waypoints_list[i].x - my_waypoints_list[0].x;
			p.y = my_waypoints_list[i].y - my_waypoints_list[0].y;
			p.z = 0.0;
		
			//printf("0: pos p:[%6.3lf %6.3lf] \n",p.x,p.y);
			vec_point.push_back(p);
		}
		
	}
	
    
	ros::Rate loop_rate(5);  // 10

	while (ros::ok())
	{
	 // 초기화  

	visualization_msgs::MarkerArray node_arr;    

	//printf("GPS flag_gps_datum_topic = %d\n",flag_gps_datum_topic );	

	
	/*
	for (const auto& point : vec_point) 
	{
			printf("vector_point: (%.2f, %.2f, %.2f)\n", point.x, point.y, point.z);
	}
    */
	for (size_t i = 0; i < vec_point.size(); i++)
	{
		Point3D o_node = vec_point[i];

		visualization_msgs::Marker node;
		node.header.frame_id = "utm"; // utm frame 기준
		node.header.stamp = ros::Time::now();
		node.type = visualization_msgs::Marker::SPHERE;
		node.id = i;
		node.action = visualization_msgs::Marker::ADD;
		node.pose.orientation.w = 1.0;
		node.pose.position.x =  o_node.x; //노드의 x 좌표
		node.pose.position.y =  o_node.y; //노드의 y 좌표
		//printf("o_node : [%6.3lf %6.3lf] \n",o_node.x ,o_node.y);
		// Points are green        
		if(i == target_goal_id)
		{
			node.color.g = 1.0;
			node.color.b = 0.0;
			node.color.a = 1.0;
		}
		else
		{
			node.color.g = 0.0;
			node.color.b = 0.7;
			node.color.a = 1.0;  
		}
		node.scale.x = 0.3;                       // marker size
		node.scale.y = 0.3;                       // marker size
		node.scale.z = 0.1;                       // marker size
		node_arr.markers.push_back(node);
	   
	}

	for (size_t i = 0; i < vec_point.size(); i++)
	{
		Point3D o_node_txt = vec_point[i];

		visualization_msgs::Marker node_txt;
		node_txt.header.frame_id = "utm"; // utm frame 기준
		node_txt.header.stamp = ros::Time::now();
		
		node_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		node_txt.id = i+1000;
		node_txt.action = visualization_msgs::Marker::ADD;
		node_txt.pose.orientation.w = 1.0;
		node_txt.text = "wp_" +std::to_string(i);
		node_txt.pose.position.x =  o_node_txt.x+ 0.5; //노드의 x 좌표
		node_txt.pose.position.y =  o_node_txt.y; //노드의 y 좌표
		//printf("pos [%6.3lf %6.3lf] \n",vec_point[i],vec_point[i].y);
		
		// Points are green
		node_txt.color.b = 1.0;
		node_txt.color.g = 1.0;
		node_txt.color.a = 1.0;
		
		node_txt.scale.x = 0.7;                       // marker size
		node_txt.scale.y = 0.7;                       // marker size
		node_txt.scale.z = 0.3;                       // marker size
		node_arr.markers.push_back(node_txt);
		
	}  
	marker_pub.publish(node_arr); 
	node_arr.markers.clear();


	path_line.header.frame_id = "utm";
	path_line.header.stamp = ros::Time::now();
	path_line.action = visualization_msgs::Marker::ADD;
	path_line.pose.orientation.w = 1.0;
	path_line.id = 201;
	path_line.type = visualization_msgs::Marker::LINE_STRIP; 
	path_line.scale.x = 0.1;
	path_line.color.g = 1.0;   // green color
	path_line.color.a = 1.0;

	if( (target_goal_id>=1) && (target_goal_id < vec_point.size()) )
	{ 
	   geometry_msgs::Point p_line;
	   p_line.x = vec_point[target_goal_id-1].x; 	
	   p_line.y = vec_point[target_goal_id-1].y;
	   path_line.points.push_back(p_line);

	   p_line.x = vec_point[target_goal_id].x; 	
	   p_line.y = vec_point[target_goal_id].y;
	   path_line.points.push_back(p_line);
	   marker_line_pub.publish(path_line);
	   path_line.points.clear();
	   
	}

	loop_rate.sleep();
	ros::spinOnce();

	}
	return 0;
}
