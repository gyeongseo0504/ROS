// 후진일때 속도 제어 코드 수정완료 2024.09.30

#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>

using namespace std;

#define MAX_angluar_velocity M_PI/10

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d, yaw_d_360;
double yaw_d_old  = 0.;
double target_yaw = 0.;
double traffic_cone_target_yaw = 0.;

double Kp                 = 0.01;
double Kd                 = 0.04;
double Ki                 = 0.00;

double error              = 0.0;
double error_d            = 0.0;
double error_sum          = 0.0;


double Kp_xte             = 0.01;
double Kd_xte             = 0.04;
double Ki_xte             = 0.00;


double Kp_vision          = 0.01;
double Kd_vision          = 0.04;
double Ki_vision          = 0.00;


double error_xte          = 0.0;
double error_xte_old        = 0.0;
double error_xte_sum      = 0.0;

double error_vision       = 0.0;
double error_vision_old   = 0.0;
double error_vision_sum   = 0.0;


double imu_heading_angle        = 0.0;                 // radain 값으로 imu에서 나오는 heading_angle(yaw)
double imu_heading_angle_d      = 0.0;                 // radain 값으로 imu에서 나오는 heading_angle(yaw) ~180 ~ 180 
double imu_heading_angle_d_360  = 0.0;                 // radain 값으로 imu에서 나오는 heading_angle(yaw)  0~360
double imu_heading_anlge_offset = 0.0;

bool   control_action_flag  = 0;
bool   use_imu = false;

int    right_angle_max = -25;
int    left_angle_max  =  25;


int    xte_right_angle_max           = -10;
int    xte_left_angle_max            =  10;

int    vision_xte_right_angle_max    = -15;
int    vision_xte_left_angle_max     =  15;


int    steering_control_mode        = -1;

double virtual_cross_track_error    = 0.0;
double vision_cross_track_error     = 0.0;

bool   yaw_control_complete_flag    = false;
double car_control_speed            = 0.0;

void car_control_speed_Callback(const std_msgs::Float32& msg)
{
    car_control_speed = msg.data;

}
void virtual_cross_track_error_Callback(const std_msgs::Float32& msg)
{
	virtual_cross_track_error = msg.data;
	control_action_flag = 1;
}

void vision_cross_track_error_Callback(const std_msgs::Float32& msg)
{
	vision_cross_track_error = msg.data;
	control_action_flag = 1;
}

void imu_yaw_Callback(const std_msgs::Float32& msg)
{
	imu_heading_angle = msg.data;
	imu_heading_angle_d = RAD2DEG(imu_heading_angle);

	imu_heading_angle_d_360 = (imu_heading_angle_d < 0) ? imu_heading_angle_d + 360 : imu_heading_angle_d;
}

void yaw_control_mode_Callback(const std_msgs::Int8& msg)
{
	steering_control_mode = msg.data;
}

void gps_heading_angle_Callback(const std_msgs::Float32& msg)
{
	yaw   = msg.data;
	yaw_d = RAD2DEG(yaw) ;
	yaw_d_360 = (yaw_d < 0) ? yaw_d + 360 : yaw_d;
}

void target_yaw_Callback(const std_msgs::Float32& msg)
{
	int     quotient  = 0  ;
	double  remainder = 0.0;
	target_yaw = msg.data ;
	//if(target_yaw < 0) target_yaw +=360;
	
	quotient  = target_yaw/360;
	target_yaw = target_yaw - 360*quotient;
	
	if(target_yaw  <-180)
	{
		target_yaw += 360;		
	}
	
	//target_yaw  =  (target_yaw >=  180) ? (target_yaw -360.0) : target_yaw;
	control_action_flag = 1;
}


void traffic_cone_target_yaw_Callback(const std_msgs::Float32& msg)
{
	int     quotient  = 0  ;
	double  remainder = 0.0;
	traffic_cone_target_yaw = msg.data ;
	//if(target_yaw < 0) target_yaw +=360;
	
	quotient                = traffic_cone_target_yaw/360;
	traffic_cone_target_yaw = traffic_cone_target_yaw - 360*quotient;
	
	if(traffic_cone_target_yaw  <-180)
	{
		traffic_cone_target_yaw += 360;		
	}
	//printf("traffic_cone_test \n");
	
	//target_yaw  =  (target_yaw >=  180) ? (target_yaw -360.0) : target_yaw;
	control_action_flag = 1;
}

double control_virtual_xte(void)
{
	double steering_angle_control = 0;
	double error_xte_d            = 0.0;
    //printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    //printf("error %6.3lf  %6.3lf \n", error1, error2);

	error_xte = virtual_cross_track_error;	
	
	error_xte_d = error_xte - error_xte_old;
	steering_angle_control = Kp_xte * error_xte + Kd_xte * error_xte_d + Ki_xte * error_xte_sum;
	
	error_xte_old = error_xte;
	
	if( steering_angle_control <= xte_right_angle_max )  steering_angle_control = xte_right_angle_max;
	if( steering_angle_control >= xte_left_angle_max )   steering_angle_control = xte_left_angle_max;
	
	if( steering_control_mode == 1)
	{ 
		printf("steering_control_mode : %d\n",steering_control_mode);
		ROS_INFO("virtual_cross_track_error : %6.3lf steer angle : %lf",virtual_cross_track_error, steering_angle_control );
	}
	
	return steering_angle_control;
	
}


double control_vision_xte(void)
{
	double steering_angle_control = 0;
	double  error_vision_d        = 0.0; 
    //printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    //printf("error %6.3lf  %6.3lf \n", error1, error2);

	error_vision = vision_cross_track_error;	
	
	error_vision_d = error_vision - error_vision_old  ;
	
	steering_angle_control = Kp_vision * error_vision + Kd_vision * error_vision_d + Ki_vision * error_vision_sum;
	
	error_vision_old = error_vision ;
	error_vision_sum = 0.0;
	
	printf("error : %6.3lf  erorr_d %6.3lf steering_angle :%6.3lf\n",error_vision,error_vision_d ,steering_angle_control);
	
	if( steering_angle_control <= vision_xte_right_angle_max )  steering_angle_control = vision_xte_right_angle_max;
	if( steering_angle_control >= vision_xte_left_angle_max )   steering_angle_control = vision_xte_left_angle_max;
	
	
	if( steering_control_mode == 7)
	{ 
		printf("steering_control_mode : %d\n",steering_control_mode);
		printf("Kp_vision : %6.3lf %6.3lf %6.3lf\n",Kp_vision,Kp_vision * error_vision ,steering_angle_control);
		ROS_INFO("vision_cross_track_error : %6.3lf steer angle : %lf",vision_cross_track_error, steering_angle_control );
	}
	
	return steering_angle_control;
	
}

double traffic_cone_control_yaw(void)
{
	double steering_angle_control = 0;
	double yaw_d1;
	//int     quotient  = 0  ;
	//double  remainder = 0.0;
	int error_d = 0;
	double error1;

    //printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    //printf("error %6.3lf  %6.3lf \n", error1, error2);

	if( use_imu == 0)
	{
		error1 = traffic_cone_target_yaw - yaw_d;	
	}
	else
	{
		error1 = traffic_cone_target_yaw - imu_heading_angle_d;	
	}
	 
	if(fabs(error1) <= 1.0)
	{
		printf("yaw control done\n\n");		
		
	}
	else
	{
	
	}
	
	if(error1 >= 180)  error1 = error1 - 360;
    if(error1 <=-180)  error1 = error1 + 360;
     
    /*       
    if( (target_yaw > 179) || (target_yaw < -179))
    {
		error1 = target_yaw - yaw_d_360;		
	
	} 		
	*/
	
    error = error1;
    //if(fabs(error1) > fabs(error2))  error = error2;
    //else error = error2;
    
	steering_angle_control = Kp * error + Kd * error_d + Ki * error_sum;
	
	error_d = error;
	yaw_d_old = yaw_d1;
	
	if( steering_angle_control <= right_angle_max )  steering_angle_control = right_angle_max;
	if( steering_angle_control >= left_angle_max )   steering_angle_control =  left_angle_max;
	if( ( use_imu == 0)  )
	{	
		printf("steering_control_mode : %d\n",steering_control_mode);
		ROS_INFO("GPS Yaw Angle : %6.3lf %6.3lf Target Yaw : %6.3lf Error : %6.3lf | steer angle %6.3lf",yaw_d,yaw_d_360,target_yaw,error,steering_angle_control );
	}
	else
	{
		printf("steering_control_mode : %d\n",steering_control_mode);
		ROS_INFO("IMU Yaw Angle : %6.3lf %6.3lf Target Yaw : %6.3lf Error : %6.3lf | steer angle %6.3lf",imu_heading_angle_d,imu_heading_angle_d_360,target_yaw,error,steering_angle_control );
	}
	
	return steering_angle_control;
}

double control_yaw(void)
{
	double steering_angle_control = 0;
	double yaw_d1;
	//int     quotient  = 0  ;
	//double  remainder = 0.0;
	int error_d = 0;
	double error1;

    //printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    //printf("error %6.3lf  %6.3lf \n", error1, error2);

	if( use_imu == 0)
	{
		error1 = target_yaw - yaw_d;
	}
	else
	{
		error1 = target_yaw - imu_heading_angle_d;
	}
	 
	if(fabs(error1) <= 1.0)
	{
		printf("yaw control done\n\n");
		
	}
	else
	{
	
	}
	
	if(error1 >= 180)  error1 = error1 - 360;
    if(error1 <=-180)  error1 = error1 + 360;
     
    /*       
    if( (target_yaw > 179) || (target_yaw < -179))
    {
		error1 = target_yaw - yaw_d_360;		
	
	} 		
	*/
	
    error = error1;
    //if(fabs(error1) > fabs(error2))  error = error2;
    //else error = error2;
    
	steering_angle_control = Kp * error + Kd * error_d + Ki * error_sum;
	
	if( car_control_speed <=0) // 진행 방향이 반대일때에는 회전을 반대로
	{
		steering_angle_control = -steering_angle_control;
	}

	error_d = error;
	yaw_d_old = yaw_d1;
	
	if( steering_angle_control <= right_angle_max )  steering_angle_control = right_angle_max;
	if( steering_angle_control >= left_angle_max )   steering_angle_control =  left_angle_max;
	if( ( use_imu == 0)  )
	{	
		printf("steering_control_mode : %d\n",steering_control_mode);
		ROS_INFO("GPS Yaw Angle : %6.3lf %6.3lf Target Yaw : %6.3lf Error : %6.3lf | steer angle %6.3lf",yaw_d,yaw_d_360,target_yaw,error,steering_angle_control );
	}
	else
	{
		printf("steering_control_mode : %d\n",steering_control_mode);
		ROS_INFO("IMU Yaw Angle : %6.3lf %6.3lf Target Yaw : %6.3lf Error : %6.3lf | steer angle %6.3lf",imu_heading_angle_d,imu_heading_angle_d_360,target_yaw,error,steering_angle_control );
	}
	
	return steering_angle_control;
}

int main(int argc, char **argv)
{
 
	double pid_output = 0.0;
	ros::init(argc, argv, "car_yaw_control_node");
	ros::NodeHandle n;

	std::string imu_yaw_angle_topic                = "/imu/heading_angle_radian";
	std::string gps_heading_angle_topic            = "/gps_heading_angle";
	std::string yaw_control_mode_topic             = "/Car_Control_Cmd/steering_control_mode";

	std::string yaw_target_topic                   = "/Car_Control_Cmd/Target_Angle";
	std::string traffic_cone_yaw_target_topic      = "/traffic_cone/heading_angle";
	std::string yaw_control_steering_output_topic  = "/Car_Control_Cmd/SteerAngle_Int16";
	std::string car_speed_control_topic            = "/Car_Control_Cmd/speed_Float32";   

	std::string cross_track_error_topic            = "/virtual_cross_track_error";
	std::string vision_cross_track_error_topic     = "/vision_xte"; 

	ros::param::get("~use_imu", use_imu);     
	ros::param::get("~use_imu", use_imu);     

	ros::param::get("~imu_yaw_angle_topic", imu_yaw_angle_topic);     
	ros::param::get("~gps_heading_angle_topic",  gps_heading_angle_topic);

	ros::param::get("~yaw_control_mode_topic" ,yaw_control_mode_topic);
	ros::param::get("~yaw_target_topic",  yaw_target_topic);   
	ros::param::get("~traffic_cone_yaw_target_topic",traffic_cone_yaw_target_topic);
	                                   
	ros::param::get("~yaw_control_steering_output_topic",  yaw_control_steering_output_topic);
	ros::param::get("~car_speed_control_topic", car_speed_control_topic);  // 후진시 방향 제어 반대로 하기 위한 topic 수신

	ros::param::get("~cross_track_error_topic",cross_track_error_topic);
	ros::param::get("~vision_cross_track_error_topic",vision_cross_track_error_topic);

	ros::param::get("~Kp", Kp);  
	ros::param::get("~Kd", Kd);  
	ros::param::get("~Ki", Ki);  

	ros::param::get("~Kp_xte", Kp_xte);  
	ros::param::get("~Kd_xte", Kd_xte);  
	ros::param::get("~Ki_xte", Ki_xte);
	
	ros::param::get("~Kp_vision", Kp_vision);  
	ros::param::get("~Kd_vision", Kd_vision);  
	ros::param::get("~Ki_vision", Ki_vision);
	
	ros::param::get("~left_angle_max", left_angle_max);
	ros::param::get("~right_angle_max", right_angle_max);
	
	ros::param::get("~xte_left_angle_max",  xte_left_angle_max);
	ros::param::get("~xte_right_angle_max", xte_right_angle_max);
	
	ros::param::get("~vision_xte_left_angle_max",  vision_xte_left_angle_max);
	ros::param::get("~vision_xte_right_angle_max", vision_xte_right_angle_max);
	

	
	
	/*
	cout << "imu_yaw_angle_topic : " << imu_yaw_angle_topic <<endl;
	cout << "gps_heading_angle_topic :" << gps_heading_angle_topic  <<endl;


	cout << "yaw_target_topic : " << yaw_target_topic <<  endl;    
	cout << "yaw_control_steering_output_topic : " << yaw_control_steering_output_topic << endl;  
	cout << "cross_track_error_topic : " <<cross_track_error_topic << endl;
	*/ 

	roll = pitch = yaw = roll_d = pitch_d = yaw_d = yaw_d_old = 0.0;

	geometry_msgs::Vector3 rpy_angle_radian;
	geometry_msgs::Vector3 rpy_angle_degree;

	std_msgs::Int16 steering_angle;


	ros::Subscriber sub_IMU_yaw_angle                  = n.subscribe(imu_yaw_angle_topic, 0, &imu_yaw_Callback);
	ros::Subscriber sub_gps_heading_angle              = n.subscribe(gps_heading_angle_topic,1,&gps_heading_angle_Callback);
	ros::Subscriber sub_car_control_speed              = n.subscribe(car_speed_control_topic, 5,car_control_speed_Callback); 

	ros::Subscriber sub_target_yaw                     = n.subscribe(yaw_target_topic, 1, &target_yaw_Callback); 
	ros::Subscriber sub_traffic_cone_target_yaw        = n.subscribe(traffic_cone_yaw_target_topic,1,&traffic_cone_target_yaw_Callback);
	ros::Subscriber sub_yaw_control_mode               = n.subscribe(yaw_control_mode_topic, 1, &yaw_control_mode_Callback);
	ros::Subscriber sub_virtual_cross_track_error      = n.subscribe(cross_track_error_topic,1, &virtual_cross_track_error_Callback);
	ros::Subscriber sub_vision_cross_track_error       = n.subscribe(vision_cross_track_error_topic,1, &vision_cross_track_error_Callback);

	ros::Publisher  yaw_control_steering_output_pub    = n.advertise<std_msgs::Int16>(yaw_control_steering_output_topic, 1);
	



	ros::Rate loop_rate(25.0); //10.0HZ
  
	while(ros::ok())
	{
		  rpy_angle_radian.x = roll;
		  rpy_angle_radian.y = pitch;
		  rpy_angle_radian.z = yaw;
		  
		  rpy_angle_degree.x = roll_d;
		  rpy_angle_degree.y = pitch_d;
		  rpy_angle_degree.z = yaw_d;
		  
		  if(control_action_flag == 1)
		  {
			if(steering_control_mode ==0)  
			{
				ROS_INFO("GPS_WP_Control_mode");
				pid_output = control_yaw();
				steering_angle.data = pid_output;
				yaw_control_steering_output_pub.publish(steering_angle);    
			}
			else if (steering_control_mode == 1)  
			{
				steering_angle.data = control_virtual_xte();
				yaw_control_steering_output_pub.publish(steering_angle);    
			}
			
			else if (steering_control_mode == 7)   // vision control mode  2023.10.07
			{
				ROS_INFO("Vision_Control_mode");			
				steering_angle.data = control_vision_xte();
				yaw_control_steering_output_pub.publish(steering_angle);    
			}
			
			else if (steering_control_mode == 8)  //traffic cone control
			{
				ROS_INFO("Traffic_Cone_Control_mode");			
				pid_output = traffic_cone_control_yaw();
				steering_angle.data = pid_output;
				yaw_control_steering_output_pub.publish(steering_angle);    
			}
			
			else
			{
				pid_output = control_yaw();
				steering_angle.data = pid_output;
				yaw_control_steering_output_pub.publish(steering_angle);    
			}
		  }
		  else
		  {
			ROS_WARN("No target_angle");  
		  }
		  
		  ros::spinOnce();
		  loop_rate.sleep();
	  }

  return 0;
}
