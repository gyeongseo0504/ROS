#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/NavSatFix.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "sensor_msgs/Imu.h"

#include <math.h>

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define Line_Follwoing_Control_Mode  1.0  //단위는 m이며 가상 line following mode에 진입  가상 line을 중심으로 1.0m 이내에 들면 ㅣine follooing mode로 steering 각도를 제어 함


#define WayPoints_NO 500
#define WayPoint_X_Tor 0.4//0.4
#define WayPoint_Y_Tor 0.3//0.5

#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1
#define Pass_Region_NO 1
#define Park_Region_NO 1

double pos_x = 0.0;
double pos_y = 0.0;

float temp_speed                 =0.0;

int    vision_steering_angle     = 0;
int    waypoint_steering_angle   = 0;
double steering_angle_base       = 0.0;
double car_steering_angle        = 0.0;
double car_speed                 = 0;
int    no_waypoints              = WayPoints_NO;
bool   lidar_obs_flag            = false;
bool   lidar_obs_flag_old        = false;
bool   obs_control_trigger_flag  = false;
bool   obs_waypoint_lock_flag    = false; 

bool   topic_gps_datum_rcv       = false;
bool   use_utm_absolute_mode     = true;
double gps_heading_angle         = 0.0;
double waypoint_line_angle       = 0.0;
double waypoint_line_angle_utm   = 0.0;
double basic_waypoint_line_angle = 0.0;

double datum_lat;
double datum_lon;
double datum_yaw;

double datum_utm_east;
double datum_utm_north;

double target_utm_x = 0.0;
double target_utm_y = 0.0; 

//init_flag
int cnt_flag           = 0;
int init_flag          = 0;
int wp_go_id           = 0;
int wp_finish_id       = 0;
int run_flag           = 0;
bool use_imu_yaw_angle = 1;

int    steering_control_method     = 2;
double steering_correction_factor  = 1.0;
double way_point_ahead_distance    = 0.5;

double cross_track_offset  = 0.0;

double roll,pitch,yaw;

double imu_roll,imu_pitch,imu_yaw;
double imu_heading_angle                  = 0.0;
double cross_track_error                  = 0.0; 
double virtual_cross_track_error          = 0.0;
double cross_track_correction_angle_limit = 10.0;



double look_ahead_distance           = 0.8;
double obstacle_avoid_heading_angle  = 0.0;
bool   enable_lidar_avoidance        = false;

double ai_max_class_id         = -1;
double ai_max_class_confidence = 0.0;

int    steering_control_mode = -1;
double location_virtual_line_sensor = 0.5;  // m 단위

// 0 : heading angle
// 1 : cross_track_error_control
// 5 : obstacle_control

 
struct Point 
{ 
	double x; 
	double y; 
	double z;
};

struct WayPoints
{
	double x;
	double y;	
	double theta;
};

struct Line_eqation
{
	double a_utm;
	double b_utm;
	double theta;
	double theta_utm;
	
};

struct Rect_Region
{
	double top;
	double bottom;
	double left;
	double right;	
};

geometry_msgs::Pose2D my_pose;
geometry_msgs::Pose2D my_pose_map;
geometry_msgs::Pose2D my_pose_odom;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::Pose2D my_pose_utm_waypoint;    // coordinate,  current waypoint to goal waypoint
geometry_msgs::Pose2D initial_gps_pose;
geometry_msgs::Pose2D obstacle_avoid_target_waypoint;
geometry_msgs::Pose2D utm_datum_pose;

struct WayPoints my_waypoints_list[WayPoints_NO];
struct WayPoints target_waypoint;
struct Line_eqation Waypoints_line_eqation[WayPoints_NO];

void run_flag_Callback(const std_msgs::Int8& flag)
{
	run_flag 				  = flag.data;
	lidar_obs_flag            = false;
    lidar_obs_flag_old        = false;
    obs_control_trigger_flag  = false;
	obs_waypoint_lock_flag    = false; 
}

//GPS의경우 UTM 좌표를 따라서 XY가 다름

void gps_utm_poseCallback(const geometry_msgs::Pose2D& msg)
{
	my_pose.x     =   msg.x;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose.y     =   msg.y;      //UTM 좌표의 경우 map 좌표와 X,Y 좌표가 90도 회전되어 있음
	my_pose.theta =   msg.theta;
	
	my_pose_map.x     =  my_pose.y;
	my_pose_map.y     =  my_pose.x;
	my_pose_map.theta =  my_pose.theta; 
	
//if(msg.theta <=0) my_pose.theta =  msg.theta + 2*M_PI;   
}

void gps_datum_Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	//printf("GPS Datum RCV!\n");
	topic_gps_datum_rcv = true;
	datum_lat = msg->x;               
	datum_lon = msg->y;
	datum_yaw = msg->z;  
}

void waypointstartIDCallback(const std_msgs::Int16& msg)
{	
	wp_go_id = msg.data; 
}

void waypointfinishIDCallback(const std_msgs::Int16& msg)
{	
	wp_finish_id = msg.data; 
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data;   // radian 으로 받을 것
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	my_pose_odom.x = (double)msg.pose.pose.position.x;
	my_pose_odom.y = (double)msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,        msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);     
 
    m.getRPY(roll, pitch, yaw);
    my_pose_odom.theta = yaw;		
}

/*
void init_waypoint_region(void)
{
	FILE *fp;
	
	fp= fopen("//home//moon//one_fifth_catkin_ws//src//waypoints//waypoints_data.txt","r");
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoint_region does not exit!");
		WayPoint_Region[0].top    =  7.6;
	    WayPoint_Region[0].bottom =  7.2;
	    WayPoint_Region[0].left   =  -4.9; 
	    WayPoint_Region[0].right  =  -5.1;
   }
   else
   {
	   fscanf(fp,"%lf %lf %lf %lf",&WayPoint_Region[0].top, &WayPoint_Region[0].bottom, &WayPoint_Region[0].left, &WayPoint_Region[0].right);
	   fclose(fp);
   }
}
*/ 

void init_waypoint(void)
{
	FILE *fp;
	int result = -10;
	fp= fopen("/home/amap/one_fifth_catkin_ws/src/waypoints/waypoints_data.txt","r");
	
	//fp = NULL;
	
	if(fp == NULL)
	{
		ROS_INFO("Waypoints_data does not exit!");
		
	    my_waypoints_list[0].x = 1;   
        my_waypoints_list[0].y = 1;
	
	    my_waypoints_list[1].x = 3;   
        my_waypoints_list[1].y = 3;
  
        my_waypoints_list[2].x = 2;   
        my_waypoints_list[2].y = 6;  		
 
        my_waypoints_list[3].x = 3;   
        my_waypoints_list[3].y = 10; 
        
        no_waypoints = 4;
        wp_finish_id = no_waypoints;
        exit(1);
   }
   else
   {
	    no_waypoints = -1;
	    do
	    {
			
			++no_waypoints;
			result = fscanf(fp,"%lf %lf  %lf",&my_waypoints_list[no_waypoints].x, &my_waypoints_list[no_waypoints].y,&my_waypoints_list[no_waypoints].theta);							   
		} while(result != EOF);
				
		wp_finish_id = no_waypoints;

		ROS_INFO("WayPoints Number %d",no_waypoints);
		for(int i=0; i<no_waypoints; i++)
		{
			ROS_INFO("WayPoints-%d : [%.2lf %.2lf %.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y, my_waypoints_list[i].theta);
		}
	
		for(int i=0; i<no_waypoints-1; i++)
		{
			double delta_x, delta_y;
			double a,b;
			double waypoint_line_angle_utm, waypoint_line_angl;
			 
			delta_x = my_waypoints_list[i+1].x - my_waypoints_list[i].x;
			delta_y = my_waypoints_list[i+1].y - my_waypoints_list[i].y;
			
			waypoint_line_angle_utm  = atan2(delta_y, delta_x);
			waypoint_line_angle      = waypoint_line_angle_utm - M_PI/2;     // map 좌표계 임  90도  	
				
			a = tan(waypoint_line_angle_utm);
			b = - a * my_waypoints_list[i].x + my_waypoints_list[i].y;
					  
			Waypoints_line_eqation[i].a_utm     = a;
			Waypoints_line_eqation[i].b_utm     = b;			
			Waypoints_line_eqation[i].theta     = waypoint_line_angle ;
			Waypoints_line_eqation[i].theta_utm = waypoint_line_angle_utm ;
			
		}
		
		Waypoints_line_eqation[no_waypoints-1] = Waypoints_line_eqation[no_waypoints-2];
	    
	    fclose(fp);
   }
}

void WaySteerControlCallback(const std_msgs::Int16& angle)
{
  waypoint_steering_angle = (int)(angle.data) ;
 
  if(waypoint_steering_angle >= MAX_R_STEER)  waypoint_steering_angle = MAX_R_STEER;
  if(waypoint_steering_angle <= MAX_L_STEER)  waypoint_steering_angle = MAX_L_STEER;  
}


void lidar_obs_detect_Callback(const std_msgs::Bool& msg)
{
	lidar_obs_flag = msg.data;
	
	if( (lidar_obs_flag_old == false) && (lidar_obs_flag == true) ) 
	{
		obs_control_trigger_flag = true;
	}
	
	lidar_obs_flag_old = lidar_obs_flag;
	//printf("lidar_obs : %d\n", obs);
}


/*
void lidar_obs_heading_angle_Callback(const std_msgs::Float32& obstacle_heading)
{
	
	obstacle_avoid_heading_angle = obstacle_heading.data;
}
* 
*/

void waypoint_tf(void) 
{
	
	double tf_waypoint_x,tf_waypoint_y; 
	double tf_angle = 0;
	
	tf_angle = waypoint_line_angle + M_PI_2;
	// waypoint 좌표계로 내 위치를 변환 함 (출발점을 기준으로 하여 목표점이 X 방향임)
	tf_waypoint_x =  my_pose.x - my_target_pose_goal_prev.x;
	tf_waypoint_y =  my_pose.y - my_target_pose_goal_prev.y;  
	//tf_waypoint_x = 0.0 ; tf_waypoint_y = -1.0;
	//printf("tf_waypoint_x tf_waypoint_y(utm) :%6.3lf , %6.3lf\n", tf_waypoint_x, tf_waypoint_y);
	my_pose_utm_waypoint.x =  tf_waypoint_x * cos(tf_angle)  +  tf_waypoint_y * sin(tf_angle);   // rotation_matrix  
	my_pose_utm_waypoint.y = -tf_waypoint_x * sin(tf_angle)  +  tf_waypoint_y * cos(tf_angle);   	; 
		
	//printf("relative my pose at waypoint tf :%6.3lf , %6.3lf \n", my_pose_utm_waypoint.x, my_pose_utm_waypoint.y);
		
}


void waypoint_inverse_tf(double pos_x, double pos_y , double x, double y)
{
	double tf_angle = 0;
	
	tf_angle = waypoint_line_angle + M_PI_2;
	
	x += my_pose_utm_waypoint.x; 
	target_utm_x =   x * cos(-tf_angle) +  y * sin(-tf_angle);   // rotation_matrix
	target_utm_y =  -x * sin(-tf_angle) +  y * cos(-tf_angle);   	
	//printf(" x, y :  %6.3lf , %6.3lf\n",x,y);
	//printf("target_utm_x t target_utm_y(waypoint) : %6.3lf,  %6.3lf , %6.3lf\n", RAD2DEG(waypoint_line_angle), target_utm_x, target_utm_y);
	
	target_utm_x += my_target_pose_goal_prev.x;
	target_utm_y += my_target_pose_goal_prev.y;
	//tf_waypoint_x =  my_pose.x + my_target_pose_goal_prev.x;
	//tf_waypoint_y =  my_pose.y + my_target_pose_goal_prev.y;  
		
	//printf("my new heading utm target point :%6.3lf , %6.3lf \n", target_utm_x, target_utm_y);		
}


void base_link_tf_utm(void)
{
	
	double waypoint_pos_base_link_x     = 0.0;
	double waypoint_pos_base_link_y     = 0.0; 
	double waypoint_pos_base_link_theta = 0.0; 
	double tf_base_map_x,tf_base_map_y; 
	double waypoint_angle, waypoint_distance;	 
	
	tf_base_map_x = -my_pose.x;   //상대좌표로 변환  no translation
	tf_base_map_y = -my_pose.y;
	
	tf_base_map_x += my_target_pose_goal.x;   //상대좌표로 변환  no translation
	tf_base_map_y += my_target_pose_goal.y;   
	
	   
    // my_pose.theta = M_PI_2;     
	waypoint_pos_base_link_y = (  tf_base_map_x * cos(my_pose.theta) + tf_base_map_y * sin(my_pose.theta) ) *-1;   // rotation_matrix 90도 회전
	waypoint_pos_base_link_x = ( -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta) ) * 1;   		
	
	waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	
	waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
              
    //ROS_INFO("                   E : %6.3lf  N : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta)); 
    //ROS_INFO(" Baselink to wp  b_x : %6.3lf  b_y : %6.3lf",waypoint_pos_base_link_x,waypoint_pos_base_link_y);  
	    
	
}

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

void init_utm_datum(void)
{
	double zone;
	
	if(topic_gps_datum_rcv == true) return;
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
		
	utm_datum_pose.x = datum_utm_east;
	utm_datum_pose.y = datum_utm_north;
	
	printf("%6.3lf %6.3lf\n",datum_lat,datum_lon);
	printf("utm_data east = %6.3lf  utm_data_north = %6.3lf \n\n", datum_utm_east, datum_utm_north);

}

visualization_msgs::Marker Draw_Marker_point(float x, float y)
{
	
	visualization_msgs::Marker marker_point;
	float x_p = 0.0, y_p = 0.0;
	double zone;
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
	
	x_p = y - datum_utm_north;
	y_p = -(x - datum_utm_east);
	
	utm_datum_pose.x = datum_utm_east;
	utm_datum_pose.y = datum_utm_north;
	
	marker_point.header.frame_id = "/map"; // utm frame 기준
    marker_point.header.stamp = ros::Time::now();
    marker_point.type = visualization_msgs::Marker::SPHERE;
    marker_point.id = 400;
    marker_point.action = visualization_msgs::Marker::ADD;
	
	marker_point.scale.x = 0.2;
	marker_point.scale.y = 0.2;
	marker_point.scale.z = 0.1;

	marker_point.color.r = 1.0;
	marker_point.color.g = 1.0;
	marker_point.color.b = 1.0;
	marker_point.color.a = 1.0;
    
	
	marker_point.pose.orientation.w = 1.0;
	
	marker_point.pose.position.x = x_p;
	marker_point.pose.position.y = y_p;
    
	return marker_point;
}

visualization_msgs::Marker Draw_Marker_point_avoid_wp(float x, float y)
{
	
	visualization_msgs::Marker marker_point;

	
	//utm_datum_pose.x ;
	//utm_datum_pose.y ;
	
	marker_point.header.frame_id = "/map"; 
    marker_point.header.stamp = ros::Time::now();
    marker_point.type = visualization_msgs::Marker::SPHERE;
    marker_point.id = 400;
    marker_point.action = visualization_msgs::Marker::ADD;
	
	marker_point.scale.x = 0.4;
	marker_point.scale.y = 0.4;
	marker_point.scale.z = 0.1;

	marker_point.color.r = 0.0;
	marker_point.color.g = 1.0;
	marker_point.color.b = 1.0;
	marker_point.color.a = 1.0;
    
	
	marker_point.pose.orientation.w = 1.0;
	
	marker_point.pose.position.x = x;
	marker_point.pose.position.y = y;
    
	return marker_point;
}



nav_msgs::Path draw_target_line(ros::Time current_time)
{
	nav_msgs::Path target_line_path1;
	double zone;
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
	   
	   
	target_line_path1.header.stamp=current_time;
    target_line_path1.header.frame_id="/utm";
    
    target_line_path1.poses.clear();
  
	geometry_msgs::PoseStamped this_pose_stamped;
     
    this_pose_stamped.pose.position.x = my_pose.x - datum_utm_east;
    this_pose_stamped.pose.position.y = my_pose.y - datum_utm_north;

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(waypoint_line_angle);
	this_pose_stamped.pose.orientation.x = goal_quat.x;
	this_pose_stamped.pose.orientation.y = goal_quat.y;
	this_pose_stamped.pose.orientation.z = goal_quat.z;
	this_pose_stamped.pose.orientation.w = goal_quat.w;
	

 
    target_line_path1.poses.push_back(this_pose_stamped);     
    this_pose_stamped.pose.position.x =  target_utm_x - datum_utm_east ;
    this_pose_stamped.pose.position.y =  target_utm_y - datum_utm_north;
 

    goal_quat = tf::createQuaternionMsgFromYaw(waypoint_line_angle);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="/utm";
    
    
    //printf("datum: %6.3lf %6.3lf", datum_lat, datum_lon);
    target_line_path1.poses.push_back(this_pose_stamped);
     
    return target_line_path1;
}

int fix = 0;
int fix1 = 0;
int fix2 = 0;

void gps1_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
    fix1 = gps_msg->status.status; 
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_DEBUG_THROTTLE(60,"No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
		
		ROS_DEBUG_THROTTLE(60,"GPS  float.");
		return;
	}
	else if(gps_msg->status.status == 2) 
	{
		ROS_INFO("GPS  FIX.");
	}
      
} 



void imu_heading_angle_Callback(const std_msgs::Float32& msg)
{
	imu_heading_angle = msg.data;	
}


void obstacle_avoid_new_waypoint_Callback(const geometry_msgs::Pose2D &msg)
{
	
	obstacle_avoid_target_waypoint.x  = -msg.y ;
	obstacle_avoid_target_waypoint.y =   msg.x ;
	
}


void ai_max_class_id_Callback(const std_msgs::Int16 &msg)
{
	ai_max_class_id = msg.data;
	
}
void ai_max_class_confidence_Callback(const std_msgs::Float32 &msg)
{
	
    ai_max_class_confidence = msg.data;
}

void speed_to_look_ahead_(double min_speed, double max_speed, double  min_look_ahead , double  max_look_ahead)
{
	// 속도에 따른 lool ahead 각도 제한
	
	
}

double K_Steering_angle(double limit_correction_angle)
{
	double correction_angle        = 0.0;
	double correction_angle_limit  = 0.0;
	double k_steering_angle        = 0.0;
	//printf("limit_correction_angle %6.3lf\n",limit_correction_angle);
	correction_angle       = steering_correction_factor * my_pose_utm_waypoint.y * -1.0;
	
	if(correction_angle <= -limit_correction_angle)
	{
		correction_angle_limit = -limit_correction_angle ;
	}
	else if(correction_angle >=  limit_correction_angle)
	{
		correction_angle_limit = limit_correction_angle ;
	}
	else
	{
		correction_angle_limit = correction_angle;
	}
		
	k_steering_angle = RAD2DEG(steering_angle_base) + correction_angle_limit;
	
	printf("\n\n");
	printf("------------------------------ K steering -------------------------------------\n");
	printf("my_pose_utm_waypoint.y %6.3lf  \n",my_pose_utm_waypoint.y);
	printf("correcton_angle  %6.3lf(%6.3lf) \n",correction_angle,correction_angle_limit);
	printf("k-steering angle %6.3lf + %6.3lf = %6.3lf \n",RAD2DEG(steering_angle_base), correction_angle_limit, k_steering_angle);
	printf("-------------------------------------------------------------------------------\n");
	printf("\n\n");
	
	
	return k_steering_angle;
}

double K_Car_heading_angle(double limit_correction_angle)
{
	double correction_angle        = 0.0;
	double correction_angle_limit  = 0.0;
	double k_car_heading_angle        = 0.0;
	printf("limit_correction_angle %6.3lf\n",limit_correction_angle);
	
	cross_track_error = my_pose_utm_waypoint.y;
	correction_angle       = steering_correction_factor * (my_pose_utm_waypoint.y-cross_track_offset) * -1.0;
	
	if(correction_angle <= -limit_correction_angle)
	{
		correction_angle_limit = -limit_correction_angle ;
	}
	else if(correction_angle >=  limit_correction_angle)
	{
		correction_angle_limit = limit_correction_angle ;
	}
	else
	{
		correction_angle_limit = correction_angle;
	}
		
	k_car_heading_angle = RAD2DEG(waypoint_line_angle) + correction_angle_limit;
	
	printf("\n\n");
	printf("------------------------------ K heading -------------------------------------\n");
	printf("my_pose_utm_waypoint.y %6.3lf  \n",my_pose_utm_waypoint.y);
	printf("correcton_angle  %6.3lf(%6.3lf) \n",correction_angle,correction_angle_limit);
	printf("k-heading angle %6.3lf + %6.3lf = %6.3lf \n",RAD2DEG(waypoint_line_angle), correction_angle_limit, k_car_heading_angle);
	printf("-------------------------------------------------------------------------------\n");
	printf("\n\n");
	
	return k_car_heading_angle;
}

void limit_steering_anlge(int steering_anlge_limit)
{
	printf("unlimit car steering angle %6.3lf %6.3lf\n",car_steering_angle, RAD2DEG(steering_angle_base));
	car_steering_angle  =  (car_steering_angle >= RAD2DEG(steering_angle_base) + steering_anlge_limit) ?  RAD2DEG(steering_angle_base) + steering_anlge_limit : car_steering_angle;
	car_steering_angle  =  (car_steering_angle <= RAD2DEG(steering_angle_base) - steering_anlge_limit) ?  RAD2DEG(steering_angle_base) - steering_anlge_limit : car_steering_angle;
}


double virtual_xte_calculation(int wp_id, double *px_utm , double *py_utm)
{
	
	// wp_id 가 0일 때 문제 해결 해야 함
	double a = Waypoints_line_eqation[wp_id-1].a_utm;
	double b = Waypoints_line_eqation[wp_id-1].b_utm;
	
	double a1 = -1/Waypoints_line_eqation[wp_id-1].a_utm; //tan(gps_heading_angle );
	double b1 = 0;	
	//double a2 = 0;
	double virtual_xte = 0;
	double utm_cross_x, utm_cross_y;
	double utm_x,utm_y;
	
	
	utm_x = my_pose.x   + location_virtual_line_sensor *cos(gps_heading_angle + M_PI/2);
	utm_y = my_pose.y   + location_virtual_line_sensor *sin(gps_heading_angle + M_PI/2);
	

	b1 = utm_y  - utm_x * a1 ;
			
	if( fabs(a1-a) < 1e-8)
	{
		return virtual_xte = 0;
	}
	
	utm_cross_x = (b-b1)/(a1-a);		
	
	utm_cross_y = a1*utm_cross_x + b1;
    
	//printf("utm_xy [%6.3lf %6.3lf] [%6.3lf %6.3lf]\n", utm_x,utm_y, utm_cross_x, utm_cross_y); 	
	//a2 = (utm_y -my_pose.y)/(utm_x -my_pose.x);
	
	//printf("location %6.3lf \n", utm_y - (a*utm_x + b) );
	//printf("utm angle : %6.3lf %6.3lf\n", tan(gps_heading_angle + M_PI/2),a2);
	
	//printf("utm_waypoint3 : %6.3lf %6.3lf \n",   my_pose.x, my_pose.y);
	//printf("utm_waypoint4 : %6.3lf %6.3lf \n\n", my_pose.x, a1 * my_pose.x  + b1);
		
	
	*px_utm = utm_cross_x;
	*py_utm = utm_cross_y;		
	
	virtual_xte = sqrt( (utm_cross_x - utm_x)*(utm_cross_x - utm_x) + (utm_cross_y - utm_y)*(utm_cross_y - utm_y) );
	
	if( (utm_y - (a*utm_x + b)) < 0)   virtual_xte =  virtual_xte*-1;
	
	/*
	printf("utm_line : %6.3lf %6.3lf \n", a,b);
	printf("utm_waypoint1 : %6.3lf %6.3lf \n", my_waypoints_list[wp_id-1].x,my_waypoints_list[wp_id-1].y);
	printf("utm_waypoint2 : %6.3lf %6.3lf \n\n", my_waypoints_list[wp_id-1].x,a*my_waypoints_list[wp_id-1].x+b);
		
	printf("utm_waypoint5 : %6.3lf %6.3lf \n", utm_x,utm_y);
	printf("utm_waypoint6 : %6.3lf %6.3lf \n\n", utm_x,a1*utm_x+b1);
	
	printf("cross point : %6.3lf %6.3lf \n", utm_cross_x, utm_cross_y);
	*/
	//printf("virtual XTE : %6.3lf \n\n",virtual_xte);
	
	return virtual_xte;
}

int main(int argc, char **argv)
{
	//FILE* error_data = fopen("/home/chan/바탕화면/Clear_Robot_catkin_ws/Steering/include/data/error_data_1.txt","a");
	char buf[2];
	ros::init(argc, argv, "onefifth_waypoints_manager_utm");

	ros::NodeHandle n;

	std_msgs::Int16      s_angle;
	std_msgs::Float32    c_speed;
	std_msgs::Float32    car_target_angle;
	std_msgs::Int16      ros_waypoint_id;
	std_msgs::Bool       lane_set;
	std_msgs::Bool	     avoid_set;
	std_msgs::String slam_reset;
	
	geometry_msgs::Pose2D gps_init_pose2d_data;

	slam_reset.data = "reset";
	look_ahead_distance = 0;

	datum_lat = datum_lon =  datum_yaw = 0.0; 


	std::string imu_heading_angle_radian_topic         = "/imu/heading_angle_radian";


	ros::param::get("~use_utm_absolute_mode", use_utm_absolute_mode);                 //utm 절대 상대좌표 사용할 경우 gps datum 필요  
	ros::param::get("~steering_control_method",steering_control_method);              //steering 제어 방법 0 , 1, 2 에 따라 다름
	ros::param::get("~steering_correction_factor",steering_correction_factor);        //y축 error에 대한 steering 보정 개수
	

	ros::param::get("~use_imu_yaw_angle",use_imu_yaw_angle);                          //yaw 제어 값에 imu를 사용할 것인지 아닌지? 사용하면 1
	ros::param::get("~imu_heading_angle_radian_topic", imu_heading_angle_radian_topic);  
	ros::param::get("~look_ahead_distance", look_ahead_distance);
	ros::param::get("~way_point_ahead_distance", way_point_ahead_distance);  
	ros::param::get("~enable_lidar_avoidance",enable_lidar_avoidance);

	ros::param::get("~cross_track_correction_angle_limit",cross_track_correction_angle_limit);
	ros::param::get("~location_virtual_line_sensor",location_virtual_line_sensor);
					
	 
	ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/W_SteerAngle_Int16",1, &WaySteerControlCallback);
	ros::Subscriber sub2 = n.subscribe("/gps/utm_pos1",1, &gps_utm_poseCallback);


	ros::Subscriber sub_gps_datum = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // front gps      
	ros::Subscriber sub3 = n.subscribe("/start_waypoint_id_no",1, &waypointstartIDCallback);
	ros::Subscriber sub4 = n.subscribe("/finish_waypoint_id_no",1, &waypointfinishIDCallback);
	ros::Subscriber sub5 = n.subscribe("/gps/heading_angle",1,&GPSHeadingAngleCallback);


	ros::Subscriber sub_imu_yaw_angle                   = n.subscribe(imu_heading_angle_radian_topic,1,&imu_heading_angle_Callback);

	ros::Subscriber sub_fix1                            = n.subscribe("/gps1/fix",1,&gps1_check_Callback);  // front gps 
	//ros::Subscriber sub_fix2 = n.subscribe("/gps2/fix",1,&gps2_check_Callback);  // rear  gps

	ros::Subscriber lidar_obs_flag_sub                  = n.subscribe("/obstacle/lidar_avoid_control_status",1, &lidar_obs_detect_Callback);
	//ros::Subscriber lidar_obs_heading_angle_sub       = n.subscribe("/Car_Control_cmd/Steer_avoidance",5, &lidar_obs_heading_angle_Callback);

	ros::Subscriber obstacle_avoid_new_waypoint_sub     = n.subscribe("/obstacle/new_wp",1,&obstacle_avoid_new_waypoint_Callback);
	ros::Subscriber ai_max_class_id_sub                 = n.subscribe("/ai/max_class_id",1,&ai_max_class_id_Callback);
	ros::Subscriber ai_max_class_confidence_sub         = n.subscribe("ai/max_class_confidence",1,&ai_max_class_confidence_Callback);

	ros::Publisher car_control_pub1                     = n.advertise<std_msgs::Int16>("/Car_Control_Cmd/xte_steerAngle_Int16", 1);
	ros::Publisher car_control_pub2                     = n.advertise<std_msgs::Float32>("/Car_Control_Cmd/speed_Float32", 1);
	ros::Publisher car_control_pub3                     = n.advertise<std_msgs::Float32>("/Car_Control_Cmd/Target_Angle", 1);

	ros::Subscriber sub_run_flag                        = n.subscribe("/wp/waypoint_run_flag",1,&run_flag_Callback);  // front gps 
	ros::Publisher target_id_pub                        = n.advertise<std_msgs::Int16>("/wp/target_id",1);
	ros::Publisher target_pos_pub                       = n.advertise<geometry_msgs::Pose2D>("/wp/target_goal", 1);
	ros::Publisher start_pos_pub                        = n.advertise<geometry_msgs::Pose2D>("/wp/target_start", 1);


	//ros::Subscriber sonar_obs_flag_sub = n.subscribe("/obstacle_flag",5, &sonar_obs_detect_Callback);
	ros::Publisher waypoint_guide_angle_pub             = n.advertise<std_msgs::Float32>("/wp/wp_line_angle", 1);  
	
	ros::Publisher marker_point_pub1                    = n.advertise<visualization_msgs::Marker>("/marker/point",1,true);
	ros::Publisher marker_point_pub2                    = n.advertise<visualization_msgs::Marker>("/marker/avoid_wp",1,true);
	ros::Publisher marker_line_heading_angle_pub        = n.advertise<visualization_msgs::MarkerArray>("/marker/heading_angle_line", 1);
	ros::Publisher target_guide_line_pub                = n.advertise<nav_msgs::Path>("/wp/target_guide_line",1, true); 
	
	ros::Publisher reset_path_pub                       = n.advertise<std_msgs::Bool>("/reset_path",1);  

	ros::Publisher cam_capture_flag_pub                 = n.advertise<std_msgs::Bool>("/usb_cam/capture_flag",1);  

	ros::Publisher cross_track_error_pub                = n.advertise<std_msgs::Float32>("/cross_track_error",1);  
	//ros::Publisher virtual_cross_track_error_pub        = n.advertise<std_msgs::Float32>("/virtual_cross_track_error",1);  
	
	ros::Publisher steering_control_mode_pub            = n.advertise<std_msgs::Int8>("/Car_Control_Cmd/steering_control_mode",1);  
	
	ros::Publisher lane_control_set_pub                 = n.advertise<std_msgs::Bool>("/flag/lane_control_set", 1);
	ros::Publisher avoid_control_set_pub                = n.advertise<std_msgs::Bool>("/flag/avoid_control_lane_set", 1);
													

	ros::Rate loop_rate(30);  // 10 

	//GSP_init_datum.data[0] = 10.;
	//GSP_init_datum.data[1] = 11.;

	long count = 0;
	int mission_flag[WayPoints_NO] = {0,};
	double pos_error_x = 0.0;
	double pos_error_y = 0.0;

	double waypoint_distance = 0.0;
	double waypoint_gap_distance = 0.0;
	
	WayPoints my_virtual_sensor_position;
	Line_eqation my_line_map;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	geometry_msgs::Pose2D pose_goal, pose_start; 
	std_msgs::Int8  steering_control_mode_msg;
	
	
	nav_msgs::Path target_line_path; 

	init_waypoint(); 

	initial_gps_pose.x = datum_lat;  //gps datum 처리 
	initial_gps_pose.y = datum_lon;  //gps datum 처리


	int vision_id = -1;
	int vision_speed_id = -1;
	int waypoint_id = 0;

	double delta_x, delta_y ;
	delta_x = delta_y = 0.0;                  

	double base_line_a, base_line_b; // waypoint line between start and target point

	if(use_utm_absolute_mode == true)
	{
		ROS_INFO(" ");
		ROS_INFO(" ");
		ROS_INFO("\nutm absolute mode");
		ROS_INFO(" ");
		ROS_INFO(" ");
		ros::Duration(3.0).sleep() ;          
	}
	else
	{
		
		ROS_INFO(" ");
		ROS_INFO(" ");
		ROS_INFO("\n\n\n\nutm relative mode\n\n\n\n");  
		ROS_INFO(" ");
		ROS_INFO(" ");
		printf("topic_gps_datum_mode  : %d\n", topic_gps_datum_rcv );
			 
		ros::Duration(2.0).sleep() ;
	}

	std_msgs::Bool data;
	data.data = true;
	reset_path_pub.publish(data);
    steering_control_method = 2;

	////////////////// Visaul Makrer for heading_angle /////////////////////////
	Point p; 
	std::vector<Point> vec_point; 
	for(int i=0; i<1; i++) 
	{ 
		  p.x = i; p.y = i; p.z = i; 
		  vec_point.push_back(p); 
	}



	visualization_msgs::MarkerArray node_link_arr_heading_angle;

	while (ros::ok())
	{	
		gps_init_pose2d_data.x     = my_waypoints_list[0].x;
		gps_init_pose2d_data.y     = my_waypoints_list[0].y;
		gps_init_pose2d_data.theta =  0;
		
		init_utm_datum();
		
		
		if(topic_gps_datum_rcv == false)
		{
		 ROS_WARN_STREAM("utm relative mode now: waiting topic GPS Datum");
			 
		} 
	
		if(run_flag == 1)
		{
			printf("run_flag : %d \n\n\n\n\n\n",run_flag);  
		
			if(obs_control_trigger_flag == true)  ROS_ERROR("Obstacle Avoidance Mode!!!");
			if(obs_waypoint_lock_flag   == true) ROS_ERROR("Obstacle wp is locked !!!");
			
		      
			if(waypoint_id!= -1)
			{
			
				my_target_pose_goal.y =  my_waypoints_list[wp_go_id].y ;//- initial_utm_pose.x;
				my_target_pose_goal.x =  my_waypoints_list[wp_go_id].x ;//- initial_utm_pose.y;
			
			  //printf("goal_pose : %6.3lf %6.3lf \n", my_target_pose_goal.x , my_target_pose_goal.y);
			  //printf("%d %d \n", wp_go_id, no_waypoints);
				
				printf("my_pose : %6.3lf %6.3lf \n", my_pose.x , my_pose.y);
				
				if(wp_go_id == 0) 
				{
		    //utm 좌표계에서 계산됨 
				if(count<=1)
				{
					delta_x = my_waypoints_list[1].x - my_waypoints_list[0].x;
					delta_y = my_waypoints_list[1].y - my_waypoints_list[0].y;
					//delta_x = my_waypoints_list[0].x - my_pose.x;   
					//delta_y = my_waypoints_list[0].y - my_pose.y;
	        	}
					waypoint_line_angle_utm = atan2(delta_y, delta_x);
					waypoint_line_angle      = waypoint_line_angle_utm - M_PI/2;     // map 좌표계 임  90도  		
					my_target_pose_goal_prev.x = my_waypoints_list[0].x - delta_x;
					my_target_pose_goal_prev.y = my_waypoints_list[0].y - delta_y;
				
					delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
					delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
					  
					
					
					if(waypoint_line_angle<= - M_PI)   waypoint_line_angle += 2*M_PI;
					if(waypoint_line_angle>=   M_PI)   waypoint_line_angle -= 2*M_PI;
					
					if(use_imu_yaw_angle == 0)	steering_angle_base =   waypoint_line_angle-gps_heading_angle;
					else                        steering_angle_base =   waypoint_line_angle-imu_heading_angle;  
					
					printf("  0:utm line_angle           :%6.3lf\n",RAD2DEG(waypoint_line_angle_utm));
					//printf("0:delta_xy                 :%6.3lf %6.3lf \n", delta_x, delta_y);      
					printf("  0:waypoint_line angle(map) :%6.3lf\n", RAD2DEG(waypoint_line_angle)); 
					//printf("  0:waypoint_line_equa(map)  :%6.3lf\n", RAD2DEG(atan(Waypoints_line_eqation[0].a) ) ); 
					
					printf("  0:GPS heading angle(map)   :%6.3lf\n", RAD2DEG(gps_heading_angle));
					printf("  0:IMU heading angle(map)   :%6.3lf %6.3lf\n", RAD2DEG(imu_heading_angle),RAD2DEG(imu_yaw));  	
					printf("  0:steer_angle_base         :%6.3lf\n", RAD2DEG(steering_angle_base) ); 	
					//printf("  0:cross_track_offset       :%6.3lf\n",cross_track_offset ); 
							
			}
			else
			{ 		  
				
				if(obs_control_trigger_flag == true)
				{
					my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x;// - initial_utm_pose.x;
					my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y;// - initial_utm_pose.y;
					
					target_waypoint.x		   = my_waypoints_list[wp_go_id].x;
					target_waypoint.y		   = my_waypoints_list[wp_go_id].y;
					
					if(obs_waypoint_lock_flag == false)
					{
						//target_waypoint.x            = obstacle_avoid_target_waypoint.x + datum_utm_east; 
						//target_waypoint.y            = obstacle_avoid_target_waypoint.y + datum_utm_north; 
						obs_waypoint_lock_flag = true;
					}
					
					
					//my_waypoints_list[wp_go_id].x = obstacle_avoid_target_waypoint.x + datum_utm_east;
					//my_waypoints_list[wp_go_id].y = obstacle_avoid_target_waypoint.y + datum_utm_north;
					
					delta_x = target_waypoint.x - my_waypoints_list[wp_go_id-1].x;   
					delta_y = target_waypoint.y - my_waypoints_list[wp_go_id-1].y;
					      
				}
				else 
				{
				
					my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x;// - initial_utm_pose.x;
					my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y;// - initial_utm_pose.y;
					
					target_waypoint.x          = my_waypoints_list[wp_go_id].x; 
					target_waypoint.y          = my_waypoints_list[wp_go_id].y;
					    
					//delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
					//delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
										
					delta_x = target_waypoint.x - my_waypoints_list[wp_go_id-1].x;   
					delta_y = target_waypoint.y - my_waypoints_list[wp_go_id-1].y;
				}
	        	printf("%3d:delta_xy                :%6.3lf %6.3lf \n", wp_go_id,delta_x, delta_y);  
				waypoint_line_angle_utm = atan2(delta_y, delta_x);	           
				waypoint_line_angle      = waypoint_line_angle_utm - M_PI/2;     // map 좌표계 임  90도  
				
				if(waypoint_line_angle<= - M_PI)   waypoint_line_angle += 2*M_PI;
				if(waypoint_line_angle>=   M_PI)   waypoint_line_angle -= 2*M_PI;
				
				if(use_imu_yaw_angle == 0)	steering_angle_base =   waypoint_line_angle - gps_heading_angle;
				else                        steering_angle_base =   waypoint_line_angle - imu_heading_angle;  
				
				
				
				//printf("%6.3lf %6.3lf\n",datum_lat,datum_lon);
				//printf("utm_data east = %6.3lf  utm_data_north = %6.3lf \n\n", datum_utm_east, datum_utm_north);
	   
				printf("%3d:utm line_angle          :%6.3lf\n",wp_go_id,RAD2DEG(waypoint_line_angle_utm));	          
				//printf("%3d:delta_xy                :%6.3lf %6.3lf \n", wp_go_id,delta_x, delta_y);      
				printf("%3d:waypoint_line angle(map):%6.3lf\n", wp_go_id, RAD2DEG(waypoint_line_angle)); 
				//printf("%3d:waypoint_line_equa(map) :%6.3lf\n", wp_go_id, RAD2DEG(atan(Waypoints_line_eqation[wp_go_id-1].a) ) ); 	
				printf("%3d:GPS heading angle(map)  :%6.3lf\n", wp_go_id, RAD2DEG(gps_heading_angle)); 
				printf("%3d:IMU heading angle(map)  :%6.3lf %6.3lf\n", wp_go_id,RAD2DEG(imu_heading_angle),RAD2DEG(imu_yaw));	
				printf("%3d:steer_angle_base        :%6.3lf\n", wp_go_id,RAD2DEG(steering_angle_base)); 	        
				//printf("%3d:cross_track_offset      :%6.3lf\n", wp_go_id,cross_track_offset ); 
			}
			
			if(wp_go_id >=1)
			{
				std_msgs::Float32 wp_line_angle;
				wp_line_angle.data = Waypoints_line_eqation[wp_go_id-1].theta_utm; 
				
				//printf("Waypoints_line_eqation[%2d] = %6.3lf \n",wp_go_id-1, RAD2DEG( Waypoints_line_eqation[wp_go_id-1].theta));
				waypoint_guide_angle_pub.publish(wp_line_angle);
			}	
			else
			{
				std_msgs::Float32 wp_line_angle;
				wp_line_angle.data = Waypoints_line_eqation[0].theta_utm; 
				waypoint_guide_angle_pub.publish(wp_line_angle);
				
			}
			
		    
			
					
			waypoint_distance = sqrt(delta_x*delta_x + delta_y*delta_y);		                            // waypoint 사이 거리 (utm 좌표계)
			waypoint_gap_distance = sqrt(delta_x*delta_x + delta_y*delta_y) - way_point_ahead_distance;     // 내위치에서 waypoint까지 거리에서 X_tor 만큼 전 거리 미리 회전해야  함
							
			pos_error_x = abs(my_pose.x - target_waypoint.x);                         // utm 좌표계에서 내위치와 목표 waypoints 간의 X축 거리
			pos_error_y = abs(my_pose.y - target_waypoint.y);                         // utm 좌표계에서 내위치와 목표 waypoints 간의 Y축 거리
					
		  
			pose_goal.x = target_utm_x;
			pose_goal.y = target_utm_y;
					
			//printf("target_utm %6.3lf  %6.3lf \n", target_utm_x -datum_utm_east,target_utm_y -datum_utm_north);
			//printf("my pose map %6.3lf  %6.3lf \n", my_pose_map.x,my_pose_map.y );
			//printf("my pose map %6.3lf  %6.3lf \n", location_virtual_line_sensor *sin(gps_heading_angle+ M_PI/2),location_virtual_line_sensor *cos(gps_heading_angle+ M_PI/2) );
			//printf("target_utm %6.3lf  %6.3lf \n", target_utm_x ,target_utm_y );
			
			//visualization_msgs::Marker temp = Draw_Marker_point(target_utm_x, target_utm_y);  // utm 좌표를 입력해야함
			//marker_point_pub1.publish(temp);
			visualization_msgs::Marker temp;
			//printf("draw wp [%6.3lf %6.3lf]\n",obstacle_avoid_target_waypoint.y,-obstacle_avoid_target_waypoint.x);
			
			temp = Draw_Marker_point_avoid_wp(target_waypoint.y-datum_utm_north,-(target_waypoint.x-datum_utm_east));
			marker_point_pub2.publish(temp);
			
			pose_goal.theta = DEG2RAD(0);
			//target_pos_pub.publish(pose_goal);
			
			//ROS_INFO("[%3d]WayPoint goal E : %6.3lf  N : %6.3lf ",wp_go_id, my_target_pose_goal.x, my_target_pose_goal.y);  
								
			waypoint_tf();  //waypoint 의 연결선을 기준으로 하여 좌표 이동, 
			base_link_tf_utm();
			
			waypoint_inverse_tf(my_pose_utm_waypoint.x, my_pose_utm_waypoint.y,look_ahead_distance ,0); // 2.0이 waypoint_line에서 현재 위치를 기준으로 2.0m 앞으로 보도록 좌표 생성 X 축이 진행 방향임
			
			printf("%3d:waypoint distance       :%6.3lf %6.3lf\n", wp_go_id, my_pose_utm_waypoint.x, my_pose_utm_waypoint.y); 			
			
			ros_waypoint_id.data  = wp_go_id;	   
			
			if( (count>=0) && ( my_pose_utm_waypoint.x  >= waypoint_gap_distance ) )
			{           
				steering_control_mode = 0;
				printf("-----------------------------\n"); 
				printf("Arrived at My WayPoint[%3d] !\n",wp_go_id); 
				printf("-----------------------------\n"); 
			    if(obs_control_trigger_flag == true)
			    {
			    
					obs_control_trigger_flag = false;
					lidar_obs_flag_old = false;
					obs_waypoint_lock_flag = false;
					wp_go_id--;
			    	ros::Duration(0.1).sleep() ;      
				}
				count = -1;
				wp_go_id++;
			} 
			
			//printf("my_pose_utm_waypoint :%6.3lf , %6.3lf \n", my_pose_utm_waypoint.x, my_pose_utm_waypoint.y);		
			
			
			
			if( ( my_pose_utm_waypoint.y  >=-Line_Follwoing_Control_Mode  ) && ( my_pose_utm_waypoint.y <= Line_Follwoing_Control_Mode )  )
			{           
				//printf("----------------------------\n"); 
				//printf(" mode set %d \n", steering_control_mode);
						
				
			}/*
			else
			{
				steering_control_mode = 2;
				steering_control_mode_msg.data =  steering_control_mode;
				steering_control_mode_pub.publish(steering_control_mode_msg);
			}
						
			*/
			if(lidar_obs_flag == 0 )
			{	
			
				/*
				if(fix1 == 2)
				{
					
					if((wp_go_id == 0) || (wp_go_id == 1) || (wp_go_id == 2) || (wp_go_id == 3) || (wp_go_id == 4)) 
					{
						c_speed.data = 1.4;   
					}
					if(wp_go_id==5 )
					{
						if(my_pose_utm_waypoint.x >= 2)  c_speed.data = 2.2; 
						else                             c_speed.data = 2.0;   
					}	
				}
				*/
				
				if(fix1 == 2)
				{
					
					if((wp_go_id >= 0) && (wp_go_id <= 50)) 
					{					
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						printf("avoid mode %d\n",lane_set.data);
						printf(" 0-5 mode set %d \n", steering_control_mode);
						
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						c_speed.data = 2.6;
						temp_speed = c_speed.data;
					}
					else if((wp_go_id > 50) && (wp_go_id < 63)){
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						printf("avoid mode %d\n",lane_set.data);
						printf(" 5-31 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						
						c_speed.data = 1.0; 
						temp_speed = c_speed.data;
					}
					else if(wp_go_id >= 63  && wp_go_id < 73)
					{				
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						printf("avoid mode %d\n",lane_set.data);
						printf(" 31 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						
						c_speed.data = 2.6;
						temp_speed = c_speed.data;
					}
					else if(wp_go_id >= 73  && wp_go_id < 90)
					{				
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						printf("avoid mode %d\n",lane_set.data);
						printf(" 31 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						
						c_speed.data = 2.0;
						temp_speed = c_speed.data;
					}
					
					else if(wp_go_id >= 90  && wp_go_id < 399)
					{
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						
						printf(" 52 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						c_speed.data = 2.6;
						temp_speed = c_speed.data;
					}
					
					else if(wp_go_id >= 399  && wp_go_id < 441)
					{
						steering_control_mode = 7;
						if(cnt_flag == 0)
						{
							lane_set.data = true;
							avoid_set.data = true;
						}
						else
						{
							lane_set.data = false;
							avoid_set.data = false;
						}
						steering_control_mode_msg.data =  steering_control_mode;
						
						printf(" 52 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						c_speed.data = 2.6;
						temp_speed = c_speed.data;
					}
					
					else if(wp_go_id >= 441  && wp_go_id < 492)
					{
						steering_control_mode = 0;
						lane_set.data = false;
						avoid_set.data = false;
						steering_control_mode_msg.data =  steering_control_mode;
						
						printf(" 52 mode set %d \n", steering_control_mode);
						steering_control_mode_pub.publish(steering_control_mode_msg);
						lane_control_set_pub.publish(lane_set);
						avoid_control_set_pub.publish(avoid_set);
						c_speed.data = 2.6;
						temp_speed = c_speed.data;
					}
					
					
					else
					{
						steering_control_mode = 0;
					}
						
				}
				
				else
				{
					ROS_ERROR("GPS NO FIX");
					steering_control_mode = 7;
					lane_set.data = true;
				    avoid_set.data = false;
					steering_control_mode_pub.publish(steering_control_mode_msg);
					lane_control_set_pub.publish(lane_set);
				    avoid_control_set_pub.publish(avoid_set);
					//c_speed.data = 0;
				}			
			}
			
			else
			{
				if (avoid_set.data = true && cnt_flag == 0)
				{
					c_speed.data = 1.0;   // obstacle detection speed (reduce speed)
					cnt_flag++;
				}
				else                       
				{
					c_speed.data = temp_speed;
				}
			}
			
			
			////////////////////////// AI 연결 신호등 인식 /////////////////////////////////
			std_msgs::Bool cap_flag;
			/*
			if(wp_go_id == 4) // 신호등 검출 구간
			{
				cap_flag.data = true ;  // capture starts				 
				cam_capture_flag_pub.publish(cap_flag);
				
				if( (ai_max_class_confidence >= 0.7) && (ai_max_class_id == 3) )  // 번호랑 confidenc 값은 조정 할 것
				{
					c_speed.data = 1.6;				
				    car_control_pub2.publish(c_speed);
				}
				else // 신호가 그린이 아니면 무조건 멈
				{
					ROS_WARN("Waiting for Green Signal !!!");
					c_speed.data = 0.0;				
				    car_control_pub2.publish(c_speed);
				}	
				
			}
			else
			{
				cap_flag.data = false ;  // capture disable				 
				cam_capture_flag_pub.publish(cap_flag);
				
			}
			*/
			
			switch(steering_control_method) // 조향 제어 방법에 대해서 선정
			{
				case 0 : 
						s_angle.data       = (int)(car_steering_angle + 0.5);
						printf("car steering angle %6.3lf\n", car_steering_angle);	
						ROS_INFO("steering_angle : %d Speed : %6.3lf \n",s_angle.data ,c_speed.data);
						break;
			
				case 1 : 
						
						car_steering_angle = K_Steering_angle(cross_track_correction_angle_limit);
						s_angle.data       = (int)(car_steering_angle + 0.5);
						printf("car steering angle %6.3lf\n", car_steering_angle);	
						ROS_INFO("steering_angle : %d Speed : %6.3lf \n",s_angle.data ,c_speed.data);
						break;
				     
				case 2: 
				default : 
						
						if ( ( my_pose_utm_waypoint.y  >=-Line_Follwoing_Control_Mode/2  ) && ( my_pose_utm_waypoint.y <= Line_Follwoing_Control_Mode/2 )  )						
			            {
							car_steering_angle = K_Car_heading_angle(cross_track_correction_angle_limit-3);
				     	}
						else if( ( my_pose_utm_waypoint.y  >=-Line_Follwoing_Control_Mode  ) && ( my_pose_utm_waypoint.y <= Line_Follwoing_Control_Mode )  )
			            {
							car_steering_angle = K_Car_heading_angle(cross_track_correction_angle_limit);
				     	}
				     	else
				     	{
							
							car_steering_angle = K_Car_heading_angle(cross_track_correction_angle_limit+3);
						}
				        
						if (enable_lidar_avoidance == true)   
						{
							if(lidar_obs_flag == 1 )		
							{
								ROS_WARN("Lidar Avoidance Control \n");
								car_steering_angle = RAD2DEG(waypoint_line_angle) + obstacle_avoid_heading_angle ;
							}
						}	
						
						car_target_angle.data  = (int)(car_steering_angle + 0.5);
				        //printf("car heading angle %6.3lf\n", car_steering_angle);	
				        std_msgs::Float32 cross_track_error_pub_msg;
				        cross_track_error_pub_msg.data = cross_track_error;
				        cross_track_error_pub.publish(cross_track_error_pub_msg);
				        
				        ROS_INFO("heading_angle : %6.3lf Speed : %6.3lf \n",car_target_angle.data  ,c_speed.data);
						break;
			         
				         	     
			}
			
			if(wp_go_id >= wp_finish_id) 
			{
				c_speed.data = 0.0;
				car_control_pub2.publish(c_speed);
				wp_go_id = wp_finish_id;
				run_flag = 0;
				ROS_INFO("WP Mission Completed");	
			}
			
			//printf("car target angle   %6.3lf\n", car_target_angle.data );
	    
	    }// if(waypoint_id!= -1)	catkin
	
	    // publish topics	
	    target_id_pub.publish(ros_waypoint_id);
	     
	    
	    //steering_control_mode = 0;
	    //printf("cout : %ld\n", count);
	    if(count>=0)
	    {
			// 제어 방식에 따른 publish 방법 고려
			printf("mode test %d\n",steering_control_mode);
			printf("steering method  %d\n",steering_control_method);
			
			
			if(steering_control_method == 2)
			{
				printf("publish target angle!\n");
				car_control_pub3.publish(car_target_angle);
				
		    }
		    else
		    {
				car_control_pub1.publish(s_angle);
			}	
		    
		    car_control_pub2.publish(c_speed);
	    }
	        
		target_line_path = draw_target_line(current_time);
		target_guide_line_pub.publish(target_line_path);
	    
		
        
		for (size_t i = 0; i < vec_point.size(); i++)
		{ 
			visualization_msgs::Marker node_link_heading_angle; 
			node_link_heading_angle.header.frame_id = "/map"; 
			node_link_heading_angle.header.stamp = ros::Time::now(); 
			node_link_heading_angle.id = 600 + 2*i; 
			node_link_heading_angle.action = visualization_msgs::Marker::ADD; 
			node_link_heading_angle.pose.orientation.w = 1.0; 
		  // Points are blue
			node_link_heading_angle.color.r = 1.0f; 
			node_link_heading_angle.color.g =  52./255.; 
			node_link_heading_angle.color.b = 153./255.; 			
			node_link_heading_angle.color.a = 1.0; 
			
			node_link_heading_angle.scale.x = 0.1; 
			node_link_heading_angle.scale.y = 0.2; 
			
			node_link_heading_angle.type = visualization_msgs::Marker::ARROW; 
			
			geometry_msgs::Point start_p, end_p; 
			start_p.x =  (my_pose.y - utm_datum_pose.y); 
			start_p.y = -(my_pose.x - utm_datum_pose.x); 
			node_link_heading_angle.points.push_back(start_p); 
			end_p.x =  start_p.x + 7.0*cos(DEG2RAD( car_steering_angle )); 
			end_p.y =  start_p.y + 7.0*sin(DEG2RAD( car_steering_angle )); 
			node_link_heading_angle.points.push_back(end_p); 
			node_link_arr_heading_angle.markers.push_back(node_link_heading_angle); 
			//printf("map end: %6.3lf  %6.3lf \n",end_p.x, end_p.y);
			//printf("start : (%6.3lf %6.3lf) (%6.3lf %6.3lf)\n", my_pose.x, my_pose.y, utm_datum_pose.x, utm_datum_pose.y);		  		  
		} 
		marker_line_heading_angle_pub.publish(node_link_arr_heading_angle);
		node_link_arr_heading_angle.markers.clear();
		
		++count;
	    	
    }//if(run_flag == 1)
    
    else //run_flag ==0
    {   printf("\n\n");
		printf("%3d:GPS heading angle(map)  :%6.3lf\n", wp_go_id,RAD2DEG(gps_heading_angle)); 
		printf("%3d:IMU heading angle(map)  :%6.3lf %6.3lf \n", wp_go_id,RAD2DEG(imu_heading_angle),RAD2DEG(imu_yaw));
		printf("enable_lidar_avoidance : %1d\n",enable_lidar_avoidance);
		printf("Waiting for run waypoint navigation!! \n");
	}
	
	loop_rate.sleep();
    ros::spinOnce();
    
  }
  return 0;
}

