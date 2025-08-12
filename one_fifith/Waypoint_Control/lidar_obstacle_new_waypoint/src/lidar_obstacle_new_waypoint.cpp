#define DEBUG 1
#define DEBUG_ROS_INFO 1 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Pose2D.h"

#include<string.h>
#include "obstacle_blob_processing.h"
          
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
// unit : m
#define Sonar_Detect_Range 0.3

#define MAX_Obstacle_dist 10 //[m]


#define LIDAR_Obstacle_avoid_angle 20
#define LIDAR_Side_Detection 0.25
#define LIDAR_Side_Detection_anlge 10

#define OFF 0
#define ON  1

int    LIDAR_Obstacle_angle;   
double LIDAR_Obstacle_distance = 4;
bool   LIDAR_Rotation_CCW;   
double Robot_Width; 
double Robot_Width_Tolerance;


struct Point 
{ 
	float x;
	float y;
	float z;
};

double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d;
int heading_angle = 0;
int initial_heading_angle = 0;
int steer_angle = 0;
int steer_angle_wall = 0;
int status_avoidance_control = OFF;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
  /*
   *   ROS_INFO( "Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
              msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
              msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
              msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    */        
      tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
      tf2::Matrix3x3 m(q);     
 
      m.getRPY(roll, pitch, yaw);
      
     // printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
              
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = (int)(360. / RAD2DEG(scan->angle_increment));
    double *obstacle;
    
    int sum=0; 
    int sum_l = 0, sum_r = 0;
    double dist_y = 0.0; 
    int max=-1;
    int max_i = -1;
    int left_wall_sum = 0;
    double left_wall_min_distance  = MAX_Obstacle_dist;
    double right_wall_min_distance = MAX_Obstacle_dist;
    int right_wall_sum = 0;
    double free_space = 0.0;
    double free_space_min_angle = 0.0;
    double free_space_min_angle_degree = 0.0;
    
    obstacle = new double[181];
    for(int i =0;i<181;i++) obstacle[i] = MAX_Obstacle_dist; 
    
   // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   // ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
   // ROS_INFO("%d angle_range, %f, %f %f", count,RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
   // printf("count %d \n", count);
    for(int i = 0; i < count; i++)
    {
		int degree = (int)RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(degree< 0) degree += 360;      
         //printf("obstacle: %3d, %6.3f \n",   degree,   scan->ranges[i]);
        if(scan->ranges[i] <= LIDAR_Obstacle_distance )
        {
		   //printf("obstacle: %3d, %6.3f \n",   degree,   scan->ranges[i]);
		   if( ( (degree>=0) && (degree <= LIDAR_Obstacle_angle)  ) || ( (degree>=360-LIDAR_Obstacle_angle) && (degree<360)  ))
		   { 
			   sum++;
	         // printf("obstacle: %3d, %6.3f %d\n",   degree,   scan->ranges[i],sum);
		   }
			
		   if( (degree>=0) && (degree<90) ) 
		   {     
			     //printf("test1: %3d, %6.3f %3d %6.3f\n",   degree,   scan->ranges[i], 90-degree ,obstacle[90-degree] );			     
			     if(LIDAR_Rotation_CCW == 1)  obstacle[90-degree] = (scan->ranges[i]);
			     else                           obstacle[degree+90] = (scan->ranges[i]);
		
		   }
		   if( (degree>=270) && (degree<360) ) 
		   { 
			 
			   //printf("test2:%3d, %6.3f %3d %6.3f\n",   degree,   scan->ranges[i], 450-degree ,obstacle[450-degree] );
			   if(LIDAR_Rotation_CCW == 1)   obstacle[450-degree] = (scan->ranges[i]);
			   else                             obstacle[degree-270]   = (scan->ranges[i]);
		   }
		   
	    }
	    
       // ROS_INFO(": [%3d, %5d]", degree, obstacle[degree] );      
    }
   /*
    for(int i =0;i<181;i++) 
    {
		printf("%3d  %6.3f\n",i, obstacle[i]);
	}
  */
    double *no_obstacle_center_position;
    CSize *Projection_Blob_Size;
    bool *Projection_Blob_available;
    int no_blob;
    
    no_obstacle_center_position = new double[50];
	Projection_Blob_Size = new CSize[50];
	Projection_Blob_available = new bool[50];
	
	memset(no_obstacle_center_position, 0, 50 * sizeof(double));
	memset(Projection_Blob_available, 0, 50 * sizeof(bool));
    
    //(double *projection, int size_projection,  threshold, double *position, int *no_pad, CSize* Projection_Blob);
     Find_ProjectData_Center_Position(obstacle, 180, MAX_Obstacle_dist,no_obstacle_center_position, &no_blob,Projection_Blob_Size);  
     
     if(DEBUG)
     {
     printf("sum = %d, Blob no %d  \n", sum,no_blob);     
    
     //printf("Wall Distance %.3lf %3.lf \n", left_wall_min_distance,right_wall_min_distance);
     }
     steer_angle_wall = 0;
     
     if(sum <= 3) 
     {
		 
		heading_angle = 0;
		
		status_avoidance_control = OFF;
		//printf("Heading Angle = %d\n",heading_angle);
        //printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
        return;
	 }
	
	free_space_min_angle = (Robot_Width_Tolerance*2 + Robot_Width) / LIDAR_Obstacle_distance;
	free_space_min_angle_degree = RAD2DEG(free_space_min_angle);  
	//free_space = LIDAR_Obstacle_distance * DEG2RAD(LIDAR_Obstacle_angle) * 2.0;  // 0.97m 정도
     
    printf("free space min angle : %6.3lf\n",free_space_min_angle_degree );      // 32.74도
      
    for(int i=0;i<no_blob;i++)
    {
		
		 int distance = fabs(Projection_Blob_Size[i].cx- Projection_Blob_Size[i].cy);
		 
		 if(distance >= free_space_min_angle_degree)
		 {
			Projection_Blob_available[i] = 1;  // 장애물 피해서 움직일 수 있는 공간 임
		 }
		 else
		 {
			Projection_Blob_available[i] = 0;  // 장애물 피해서 움직일 수 없는 공간 임 
		 }
		 
		 if(max <=  distance)
		 {
			 max = distance ;
			 max_i = i;
		 }
		  printf("%d : [%3d %3d] avaiability : %d\n", i, Projection_Blob_Size[i].cx,Projection_Blob_Size[i].cy,Projection_Blob_available[i]);
	}
	 
	// printf("%d %d [%3d %3d]\n", max_i, max,Projection_Blob_Size[max_i].cx,Projection_Blob_Size[max_i].cy);
	 
     //double temp_angle  = 90-(max,Projection_Blob_Size[max_i].cx+Projection_Blob_Size[max_i].cy)/2;  //need to check
   
    int min_value = 360; 
    int min_angle = 360; 
    int min_i = 51;
    double temp_angle  = 720;
    for(int i=0;i<no_blob;i++)
    { 
		
		if(Projection_Blob_available[i] == 1 )
		{
			printf("%d [%3d %3d]\n", i, Projection_Blob_Size[i].cx,Projection_Blob_Size[i].cy);
	 
	        temp_angle = ( Projection_Blob_Size[max_i].cx +  Projection_Blob_Size[max_i].cy ) /2.0;
	        
			if(min_value >= abs(90.0 - Projection_Blob_Size[i].cx) )
			{
				min_value = abs(90.0 - Projection_Blob_Size[i].cx);
				min_angle = Projection_Blob_Size[i].cx;
				min_i = i;
			}
			if(min_value >= abs(90.0 - Projection_Blob_Size[i].cy) )
			{
				min_value = abs(90.0 - Projection_Blob_Size[i].cy);
				min_angle = Projection_Blob_Size[i].cy;
				min_i = i;
			}
		}
	}
	
	printf("temp_angle %6.3lf,  min_angle = %3d \n", temp_angle, min_angle);  
  
    if( (temp_angle >= 0)   && (temp_angle <90) )
    {
		
		heading_angle= 90 - min_angle + LIDAR_Obstacle_avoid_angle;

	}
	 
	if( (temp_angle >= 90)   && (temp_angle <190) )
    {
		
		heading_angle = 90- min_angle - LIDAR_Obstacle_avoid_angle;
	 
	}
     		
     if( (temp_angle<= 90)   &&(temp_angle >=-90))
     {
		status_avoidance_control = ON;
		steer_angle_wall = 0;		
	 }    
	 
	 if(no_blob == 0)   heading_angle =0;
	 
     printf("Heading Angle = %d\n",heading_angle);
     printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
    
     delete []obstacle;
     delete []no_obstacle_center_position;
     delete []Projection_Blob_Size;
     delete []Projection_Blob_available;
}

int main(int argc, char **argv)
{
  char buf[2];
  ros::init(argc, argv, "lidar_obstacle_lane_control");
  
  LIDAR_Obstacle_angle     =   20;
  LIDAR_Obstacle_distance  =  1.2;
  LIDAR_Rotation_CCW       =  1;
  Robot_Width              =  0.3;
  Robot_Width_Tolerance   =  0.10;
  
  ros::NodeHandle n;  
  std::string lidar_topic = "/scan";
  std::string imu_topic = "/handsfree/imu";
  std::string odom_pub_topic = "/odom";
//  std::string init_heading_angle = "/scan";
  std::string status_avoid_contol_topic ="/lidar_avoid_control_status"; 
  
  char frameid[] ="/sonar_range";
  
  /*other*/
  ros::param::get("~lidar_topic", lidar_topic);
  ros::param::get("~imu_topic", imu_topic); 
  ros::param::get("~odom_pub_topic", odom_pub_topic);
  ros::param::get("~status_void_contol", status_avoid_contol_topic);
  ros::param::get("~LIDAR_Obstacle_angle",     LIDAR_Obstacle_angle); //obstacle avoidance의 검출 앵글 -20~20
  ros::param::get("~LIDAR_Obstacle_distance",  LIDAR_Obstacle_distance); //obstacle avoidance의 검출 반경 1.2m
  ros::param::get("~LIDAR_Rotation_CCW",       LIDAR_Rotation_CCW);
  ros::param::get("~Robot_Width",              Robot_Width); 
  ros::param::get("~Robot_Width_Tolerance",    Robot_Width_Tolerance); 
  

  ros::Subscriber sub_IMU = n.subscribe(imu_topic, 20, imuCallback);
  ros::Subscriber sub_lidar = n.subscribe<sensor_msgs::LaserScan>(lidar_topic, 10, &scanCallback);
 // ros::Subscriber sub_sonar1 = n.subscribe<sensor_msgs::Range>(sonar1_topic, 1000, &sonar1Callback);
 // ros::Subscriber sub_sonar2 = n.subscribe<sensor_msgs::Range>(sonar2_topic, 1000, &sonar2Callback);
 // ros::Subscriber sub_sonar3 = n.subscribe<sensor_msgs::Range>(sonar3_topic, 1000, &sonar3Callback);
  
  ros::Publisher lidar_boat_control_pub1 = n.advertise<std_msgs::Int16>("/Boat_Control_cmd/lidar_steerAngle_Int16", 10);
  ros::Publisher lidar_boat_control_pub2 = n.advertise<std_msgs::Int16>("/Boat_Control_cmd/lidar_Speed_Int16", 10);
  ros::Publisher obstacle_control_status_pub = n.advertise<std_msgs::Int8>(status_avoid_contol_topic, 10);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("/avoid_line", 1);


  printf("LIDAR_Obstacle_angle : %3d \n" , LIDAR_Obstacle_angle);
  printf("LIDAR_Obstacle_distance : %4.2lf \n" , LIDAR_Obstacle_distance);
  printf("LIDAR_Rotation_CCW : %1d \n" , LIDAR_Rotation_CCW);
  printf("Robot_Width : %4.2lf \n",Robot_Width);
     
     
  ////////////////  pid control //////////////////////
  double error, error_old, error_d, error_sum;
  double pi_gain,pd_gain,p_gain;
  
  
  error = error_old = error_d = error_sum = 0.0;
  p_gain = 1.7;
  pd_gain = 3; 
  
  ////////////////   imu _sensor //////////////////////
  sensor_msgs::Imu imu_data;
  roll = pitch = yaw = 0.0;
  ////////////////   sonar _sensor //////////////////////
  sensor_msgs::Range sonar_msg;
  sonar_msg.header.frame_id =  frameid;
  sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg.field_of_view = (30.0/180.0) * 3.14;
  sonar_msg.min_range = 0.0;
  sonar_msg.max_range = 1.50;  //[unit :m]
  
  
  //////////////// Waypoint /////////////////////
  
  geometry_msgs::Pose2D  new_waypoint_obstacle;
  
  ////////////////// Visaul Makrer /////////////////////////
  Point p; 
  std::vector<Point> vec_point; 
  for(int i=0; i<1; i++) 
  { 
	  p.x = i; p.y = i; p.z = i; 
	  vec_point.push_back(p); 
  }

  visualization_msgs::MarkerArray node_link_arr;
 
   

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  
 
  ros::Rate loop_rate(20);  // 10
  
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     
    
  
     for (size_t i = 0; i < vec_point.size(); i++)
     { 
		  visualization_msgs::Marker node_link; 
		  node_link.header.frame_id = "/base_link"; 
		  node_link.header.stamp = ros::Time::now(); 
		  node_link.id = i*10+650; 
		  node_link.action = visualization_msgs::Marker::ADD; 
		  node_link.pose.orientation.w = 1.0; 
		  // Points are blue 
		  node_link.color.r = 0.0f; 
		  node_link.color.g = 0.0f; 
		  node_link.color.b = 1.0f; 
		  node_link.color.a = 1.0; 
		  node_link.scale.x = 0.2; 
		  node_link.scale.y = 0.3; 
		  node_link.type = visualization_msgs::Marker::ARROW; 
		  geometry_msgs::Point start_p, end_p; 
		  start_p.x = 0; 
		  start_p.y = 0; 
		  node_link.points.push_back(start_p); 
		  end_p.x = 6.0*cos(DEG2RAD(heading_angle)); 
		  end_p.y = 6.0*sin(DEG2RAD(heading_angle)); 
		  node_link.points.push_back(end_p); 
		  node_link_arr.markers.push_back(node_link); 		  		  
    } 
    marker_pub.publish(node_link_arr);
    node_link_arr.markers.clear();
    
    double angle_temp = (double)heading_angle;
    
    printf("heading angle %3d\n", heading_angle);
    
    error = angle_temp ;
	error_d = error - error_old;
	
	steer_angle = (int)( p_gain * error + pd_gain * error_d  + 0.5   );
	
	printf("steer angle %d \n", steer_angle);
		
	std_msgs::Int16 s_angle;
	s_angle.data =  -steer_angle%360;
	lidar_boat_control_pub1.publish(s_angle);
	
	
	std_msgs::Int8 avoidance_control_status;
	avoidance_control_status.data = status_avoidance_control ;
	obstacle_control_status_pub.publish(avoidance_control_status);
	status_avoidance_control = OFF;
		
	error_old = error;
    loop_rate.sleep();
    ros::spinOnce();
    ++count;
  }
  
  return 0;
}



