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
#include <math.h>

#define MAX_L_STEER -30
#define MAX_R_STEER 30
#define STEER_NEUTRAL_ANGLE 0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define Line_Follwoing_Control_Mode  0.4  //단위는 m이며 가상 line following mode에 진입  가상 line을 중심으로 0.4m 이내에 들면 ㅣine follooing mode로 제어 함


#define WayPoints_NO 61
#define WayPoint_X_Tor 0.3//0.4
#define WayPoint_Y_Tor 0.5//0.5



#define V_Region_NO  2
#define V_Speed_Region_NO 3
#define W_Region_NO  1
#define Pass_Region_NO 1
#define Park_Region_NO 1

double pos_x = 0.0;
double pos_y = 0.0;

int vision_steering_angle = 0;
int waypoint_steering_angle = 0;
int car_speed = 0;
int no_waypoints = WayPoints_NO;
int lidar_obs = 0;
bool sonar_obs = 0;

bool topic_gps_datum_rcv = false;
bool use_utm_absolute_mode = true;
double gps_heading_angle = 0.0;
double waypoint_line_angle = 0.0;

double datum_lat;
double datum_lon;
double datum_yaw;

double datum_utm_east;
double datum_utm_north;

double target_utm_x = 0.0;
double target_utm_y = 0.0; 

//init_flag
int init_flag = 0;
int wp_go_id = 0;
int wp_finish_id = 0;
int run_flag = 0;

double roll,pitch,yaw;

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
};

struct Rect_Region
{
	double top;
	double bottom;
	double left;
	double right;	
};

geometry_msgs::Pose2D my_pose;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::Pose2D my_pose_utm_waypoint;    // coordinate,  current waypoint to goal waypoint
geometry_msgs::Pose2D initial_utm_pose;

struct Rect_Region Vision_Region[V_Region_NO];
struct Rect_Region Vision_Speed_Region[V_Region_NO];
struct Rect_Region WayPoint_Region[W_Region_NO];
struct WayPoints my_waypoints_list[WayPoints_NO];
struct Rect_Region Passing_Region[Pass_Region_NO];
struct Rect_Region Parking_Region[Park_Region_NO];


void run_flag_Callback(const std_msgs::Int8& flag)
{
	run_flag = flag.data;
}


//GPS의경우 UTM 좌표를 따라서 XY가 다름

void gps_utm_poseCallback(const geometry_msgs::Pose2D& msg)
{
	my_pose.x     =  msg.x;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.y     =  msg.y;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
	my_pose.theta =  msg.theta;
	
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
	wp_go_id  = msg.data; 
}

void waypointfinishIDCallback(const std_msgs::Int16& msg)
{	
	wp_finish_id  = msg.data; 
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
	gps_heading_angle = msg.data;   // radian 으로 받을 것
}

void odomCallback(const nav_msgs::Odometry& msg)
{
	my_pose.x = (double)msg.pose.pose.position.x;
	my_pose.y = (double)msg.pose.pose.position.y;
	
	tf2::Quaternion q(
        msg.pose.pose.orientation.x,        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,        msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);     
 
    m.getRPY(roll, pitch, yaw);
    my_pose.theta = yaw;		
}

void init_waypoint_region(void)
{
	FILE *fp;
	
	fp= fopen("//home//amap//waypoints//waypoint_region.txt","r");
	
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
 
void init_waypoint(void)
{
	FILE *fp;
	
	fp= fopen("//home//amap-cleanbot//waypoints//waypoints_data.txt","r");
	
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
   }
   else
   {
	    fscanf(fp,"%d",&no_waypoints);
	    
	    wp_finish_id = no_waypoints;
	  
	    for(int i=0; i<no_waypoints; i++)
	    {
			fscanf(fp,"%lf %lf",&my_waypoints_list[i].x, &my_waypoints_list[i].y);
	    }
	  
	    ROS_INFO("WayPoints Number %d",WayPoints_NO);
	    for(int i=0; i<no_waypoints; i++)
	    {
			ROS_INFO("WayPoints-%d : [%.2lf]%.2lf]",i,my_waypoints_list[i].x,my_waypoints_list[i].y);
	    }
	    fclose(fp);
   }
}

void WaySteerControlCallback(const std_msgs::Int16& angle)
{
  waypoint_steering_angle = (int)(angle.data) ;
 
  if(waypoint_steering_angle >= MAX_R_STEER)  waypoint_steering_angle = MAX_R_STEER;
  if(waypoint_steering_angle <= MAX_L_STEER)  waypoint_steering_angle = MAX_L_STEER;  
}


void lidar_obs_detect_Callback(const std_msgs::Int8& msg)
{
	lidar_obs = msg.data;
	//printf("lidar_obs : %d\n", obs);
}

void sonar_obs_detect_Callback(const std_msgs::Bool& msg)
{
	sonar_obs = msg.data;
	//printf("sonar_obs : %d\n", obs);
}

void waypoint_tf(void)
{
	double x,y;
	double tf_waypoint_x,tf_waypoint_y; 
	x=y=0;
	
	tf_waypoint_x =  my_pose.x - my_target_pose_goal_prev.x;
	tf_waypoint_y =  my_pose.y - my_target_pose_goal_prev.y;  
		
	my_pose_utm_waypoint.x =  tf_waypoint_x * cos(-waypoint_line_angle) -  tf_waypoint_y * sin(-waypoint_line_angle);   // rotation_matrix
	my_pose_utm_waypoint.y =  tf_waypoint_x * sin(-waypoint_line_angle) +  tf_waypoint_y * cos(-waypoint_line_angle);   	; 
		
	printf("relative my pose at waypoint tf :%6.3lf , %6.3lf \n", my_pose_utm_waypoint.x, my_pose_utm_waypoint.y);
		
}


void waypoint_inverse_tf(double x, double y)
{
	target_utm_x =  x * cos(waypoint_line_angle) -  y * sin(waypoint_line_angle);   // rotation_matrix
	target_utm_y =  x * sin(waypoint_line_angle) +  y * cos(waypoint_line_angle);   	
	
	target_utm_x += my_target_pose_goal_prev.x;
	target_utm_y += my_target_pose_goal_prev.y;
	//tf_waypoint_x =  my_pose.x + my_target_pose_goal_prev.x;
	//tf_waypoint_y =  my_pose.y + my_target_pose_goal_prev.y;  
		
	printf("my new heading utm target point :%6.3lf , %6.3lf \n", target_utm_x, target_utm_y);		
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
	waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
	waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);   	
	
	waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);	
	waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
    
    ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta)); 
    ROS_INFO(" b_x : %6.3lf  b_y : %6.3lf",waypoint_pos_base_link_x,waypoint_pos_base_link_y);  
	    
	
}

void wgs2utm(double lat, double lon, int zone , double& east, double& north){
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
    target_line_path1.header.frame_id="map";
    
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
    this_pose_stamped.header.frame_id="map";
    
    
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




int main(int argc, char **argv)
{
  //FILE* error_data = fopen("/home/chan/바탕화면/Clear_Robot_catkin_ws/Steering/include/data/error_data_1.txt","a");
  char buf[2];
  ros::init(argc, argv, "cleanbot_race_waypoints_manager_utm");

  ros::NodeHandle n;
  
  std_msgs::Int16 s_angle;
  std_msgs::Int16 c_speed;
  std_msgs::Int16 ros_waypoint_id;
  std_msgs::String slam_reset;
  
  geometry_msgs::Pose2D gps_init_pose2d_data;

  slam_reset.data = "reset";
  
  datum_lat = datum_lon =  datum_yaw = 0.0; 
   
  ros::Subscriber sub1 = n.subscribe("/Car_Control_cmd/W_SteerAngle_Int16",10, &WaySteerControlCallback);
  ros::Subscriber sub2 = n.subscribe("/gps/utm_pos1",5, &gps_utm_poseCallback);
    
  ros::param::get("~use_utm_absolute_mode", use_utm_absolute_mode);        //2개의 GPS를 사용할 경우 yaw값은 2개의 GPS로 부터 계산함   
  //ros::param::get("~use_utm_absolute_mode", use_utm_absolute_mode);
  
    
  ros::Subscriber sub_gps_datum = n.subscribe("/gps/datum",1,&gps_datum_Callback);  // front gps      
  ros::Subscriber sub3 = n.subscribe("/start_waypoint_id_no",1, &waypointstartIDCallback);
  ros::Subscriber sub4 = n.subscribe("/finish_waypoint_id_no",1, &waypointfinishIDCallback);
  ros::Subscriber sub5 = n.subscribe("/gps_heading_angle",1,&GPSHeadingAngleCallback);
 
  ros::Subscriber sub_fix1 = n.subscribe("/fix",1,&gps1_check_Callback);  // front gps 
  ros::Subscriber sub_run_flag = n.subscribe("/waypoint_run_flag",1,&run_flag_Callback);  // front gps 
  
  //ros::Subscriber sub_fix2 = n.subscribe("/gps/fix2",1,&gps2_check_Callback);  // rear  gps
 
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 1);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 1);
  ros::Publisher target_id_pub    = n.advertise<std_msgs::Int16>("target_id",1);
  ros::Publisher target_pos_pub   = n.advertise<geometry_msgs::Pose2D>("/target_goal", 1);
  ros::Publisher start_pos_pub   = n.advertise<geometry_msgs::Pose2D>("/target_start", 1);
  ros::Subscriber lidar_obs_flag_sub = n.subscribe("/obs_flag",5, &lidar_obs_detect_Callback);
  //ros::Subscriber sonar_obs_flag_sub = n.subscribe("/obstacle_flag",5, &sonar_obs_detect_Callback);
  
  ros::Publisher target_guide_line_pub = n.advertise<nav_msgs::Path>("/target_guide_line",1, true);
   
  ros::Rate loop_rate(5);  // 10 

  //GSP_init_datum.data[0] = 10.;
  //GSP_init_datum.data[1] = 11.;

   
  long count = 0;
  int mission_flag[WayPoints_NO] = {0,};
  double pos_error_x = 0.0;
  double pos_error_y = 0.0;

  double waypoint_distance = 0.0;
  double waypoint_gap_distance = 0.0;
  

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
    
  geometry_msgs::Pose2D pose_goal, pose_start; 
  
  nav_msgs::Path target_line_path; 
    
  
  init_waypoint(); 
  
  initial_utm_pose.x = datum_lat;  //gps datum 처리 
  initial_utm_pose.y = datum_lon;  //gps datum 처리

  
  int vision_id = -1;
  int vision_speed_id = -1;
  int waypoint_id = 0;
 
  double start_x = 0.;  
  double start_y = 0.;
  //init_waypoint_region();
  
  double delta_x, delta_y ;
  delta_x = delta_y = 0.0;
  
  double base_line_a, base_line_b; // waypoint line between start and target point
   
   
  if(count == 0)
  {
	start_x = my_pose.x;   start_y = my_pose.y;
	start_x = 0;   start_y = 0;
  }  
 
  
  if(use_utm_absolute_mode == true)
  {
	     ROS_INFO(" ");
	     ROS_INFO(" ");
	     ROS_INFO("nutm absolute mode");
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
	     /*
	     while(topic_gps_datum_rcv == false)
	     {
			 ROS_WARN_STREAM("utm relative mode now: waiting topic GPS Datum");
			 
		 }
	     */
	     ros::Duration(2.0).sleep() ;
  }
 
  while (ros::ok())
  {	
    
    gps_init_pose2d_data.x     = my_waypoints_list[0].x;
    gps_init_pose2d_data.y     = my_waypoints_list[0].y;
    gps_init_pose2d_data.theta =  0;
        
    
    if(run_flag == 1)
    {
		printf("run_flag : %d \n\n\n\n\n\n",run_flag);        
        if(waypoint_id!= -1)
        {	
		
	    //check_inside_waypoint(1);
		//ROS_INFO("WayPoint Number : %d", no_waypoints);
	    //ROS_INFO("WayPoint Region : %2d",waypoint_id);
	    
	     my_target_pose_goal.x = my_waypoints_list[wp_go_id].x ;//- initial_utm_pose.x;
	     my_target_pose_goal.y = my_waypoints_list[wp_go_id].y ;//- initial_utm_pose.y;
	    
	    
	  //printf("goal_pose : %6.3lf %6.3lf \n", my_target_pose_goal.x , my_target_pose_goal.y);
	  //printf("%d %d \n", wp_go_id, no_waypoints);
	    
		   if(wp_go_id == 0) 
		   {
			  delta_x = my_waypoints_list[1].x - my_waypoints_list[0].x;
	          delta_y = my_waypoints_list[1].y - my_waypoints_list[0].y;
	        			
			  my_target_pose_goal_prev.x = my_waypoints_list[0].x - delta_x;
	          my_target_pose_goal_prev.y = my_waypoints_list[0].y - delta_y;
	        
	          delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
	          delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
	          waypoint_line_angle = atan2(delta_y, delta_x);
	     
	       // printf("start_pose : %6.3lf %6.3lf \n", start_x, start_y);   
	       //printf("2: angle %lf\n", RAD2DEG(waypoint_line_angle)); 			
		    }
		    else
	        { 		  
	           my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x;// - initial_utm_pose.x;
	           my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y;// - initial_utm_pose.y;
	        
	           delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
	           delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
	           waypoint_line_angle = atan2(delta_y, delta_x);
	        
	          // printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	        
		    }
				
		waypoint_distance = sqrt(delta_x*delta_x + delta_y*delta_y);		
		waypoint_gap_distance = sqrt(delta_x*delta_x + delta_y*delta_y) - WayPoint_X_Tor;
	    //printf("gap : %6.3lf  \n",waypoint_gap_distance);
				
		//printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle)); 	
		
			    
	    pos_error_x = abs(my_pose.x - my_waypoints_list[wp_go_id].x);
	    pos_error_y = abs(my_pose.y - my_waypoints_list[wp_go_id].y); 
	   	
	   	
	   	// publish goal waypoint pose2D     
	    //pose_goal.x =  my_waypoints_list[wp_go_id].x ;;//- initial_utm_pose.y;    // umt coordinate
	    //pose_goal.y =  my_waypoints_list[wp_go_id].y;; // + initial_utm_pose.x);   // umt coordinate 
	    
	    pose_goal.x = target_utm_x;
	    pose_goal.y = target_utm_y;
	    
	    pose_goal.theta = DEG2RAD(0);
	    target_pos_pub.publish(pose_goal);
	    
	    ROS_INFO("[%3d]WayPoint goal X : %6.3lf  goal Y : %6.3lf ",wp_go_id, my_target_pose_goal.x, my_target_pose_goal.y);  
	    
	    
	    waypoint_tf();
	    base_link_tf_utm();
	    
	    waypoint_inverse_tf(2.0 + my_pose_utm_waypoint.x  , 0);
	       
	    // publish start waypoint pose2D
        pose_start.x = my_target_pose_goal_prev.x;
        pose_start.y = my_target_pose_goal_prev.y;
        pose_start.theta = DEG2RAD(0);
        start_pos_pub.publish(pose_start);   
  
	    ros_waypoint_id.data  = wp_go_id;	      

	    //printf("d_x %6.3lf d_y %6.3lf \n", delta_x,delta_y);
	    //printf("waypoint_distance  %6.3lf \n", waypoint_distance);
	   
	   
	    //if( (count>=0) && ( my_pose_utm_waypoint.x  >=(waypoint_distance -  WayPoint_X_Tor) ) && ( my_pose_utm_waypoint.x <=(waypoint_distance +  WayPoint_X_Tor) )  )
	       if( (count>=0) && ( my_pose_utm_waypoint.x  >=(waypoint_distance -  WayPoint_X_Tor) ) )
           {           
	          printf("-----------------------------\n"); 
              printf("Arrvied at My WayPoint[%3d] !\n",wp_go_id); 
              printf("-----------------------------\n"); 
           
           //ros::Duration(4.0).sleep() ;      
	          count = -2;
	          wp_go_id++;
	        } 
	     
	    printf("relative my pose at waypoint tf :%6.3lf , %6.3lf \n", my_pose_utm_waypoint.x, my_pose_utm_waypoint.y);
	     
	    
	       if( ( my_pose_utm_waypoint.y  >=-Line_Follwoing_Control_Mode  ) && ( my_pose_utm_waypoint.y <= Line_Follwoing_Control_Mode )  )
           {           
	     
	          printf("----------------------------\n"); 
              printf("Line Following Control Mode\n"); 
              printf("----------------------------\n"); 	     
	       }
	    
	    s_angle.data = waypoint_steering_angle;
	    
	        if(lidar_obs == 1 )
	    	{
			    c_speed.data =0;
		    }
		    else
		    {
		
	         	if(fix1 == 2)
		        {
			        c_speed.data =800;
	         	}
		        else
		        {
			        c_speed.data = 0;
		        }
		
		    }
				
	        if(wp_go_id >= wp_finish_id) 
	        {
			    c_speed.data = 0;
			    wp_go_id = wp_finish_id;
			    ROS_INFO("WP Mission Completed");	
	        }
	    }// if(waypoint_id!= -1)	catkin
	
	    // publish topics	
	    target_id_pub.publish(ros_waypoint_id);
	    ROS_INFO("steering_angle : %d Speed : %d \n",s_angle.data ,c_speed.data);
	        if(count>=2)
	        {
	             car_control_pub1.publish(s_angle);
	             car_control_pub2.publish(c_speed);
	        }
	        
	    target_line_path = draw_target_line(current_time);
	    target_guide_line_pub.publish(target_line_path);
	    	
    }//if(run_flag == 1)
    else
    {
		printf("Waiting for run waypoint navigation!! \n");
	}
	
	loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  return 0;
}

