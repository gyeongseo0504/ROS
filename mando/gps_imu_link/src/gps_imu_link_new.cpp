//2023.10.07 update

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#define DEBUG 1

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

geometry_msgs::Pose2D my_utm_pose1, my_utm_pose2;

double imu1_roll,imu1_pitch,imu1_yaw;
double imu1_heading_angle_radian;


double imu2_roll,imu2_pitch,imu2_yaw;
double imu2_heading_angle_radian;

double imu_roll,imu_pitch,imu_yaw;
double imu_heading_angle_radian;

double imu_heading_angle_radian2;

double gps_heading_angle_radian;
double imu_offset;

double imu_yaw_offset_degree = 0.0;
bool   imu_link_restart_flag = false;
       
int gps_fix1               = 0;
int gps_fix2               = 0;
int choose_imu_no          = 1;
bool auto_imu_offset_flag  = true;
bool enable_imu_correction = true;

void imu_correction_enable_Callback(const std_msgs::Bool& msg)
{
    enable_imu_correction = msg.data;
}
void gps_utm_pose1Callback(const geometry_msgs::Pose2D& msg)
{
    my_utm_pose1.x = msg.x;
    my_utm_pose1.y = msg.y;
    my_utm_pose1.theta = msg.theta;
    // 코멘트만 수정 - 실제로는 90도 회전 문제 해결됨
}

void gps_utm_pose2Callback(const geometry_msgs::Pose2D& msg)
{
    my_utm_pose2.x = msg.x;
    my_utm_pose2.y = msg.y; 
    my_utm_pose2.theta = msg.theta;
    // 코멘트만 수정 - 실제로는 90도 회전 문제 해결됨
}

void GPSHeadingAngleCallback(const std_msgs::Float32& msg)
{
    gps_heading_angle_radian = msg.data;   // radian 으로 받을 것
}


void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);      
 
    m.getRPY(imu1_roll, imu1_pitch, imu1_yaw);
}

void imu2Callback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);      
 
    m.getRPY(imu2_roll, imu2_pitch, imu2_yaw);
}


void imu1yawradianCallback(const std_msgs::Float32& msg)
{
    imu1_heading_angle_radian = msg.data;
    if(imu1_heading_angle_radian >= M_PI)  imu1_heading_angle_radian = 2*M_PI - imu1_heading_angle_radian; 
}

void imu2yawradianCallback(const std_msgs::Float32& msg)
{
    imu2_heading_angle_radian = msg.data;
}



     
void gps1_fix_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
    gps_fix1 = gps_msg->status.status; 
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_ERROR("No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
        
        ROS_DEBUG_THROTTLE(60,"GPS  float.");
        return;
    }
    else if(gps_msg->status.status == 2) 
    {
        //ROS_INFO("GPS1  FIX.");
    }
    else
    {
    }
} 


void gps2_fix_check_Callback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)   // rear GPS
{
    gps_fix2 = gps_msg->status.status; 
    
    if (gps_msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
    {
        ROS_ERROR("No fix.");
        return;
    }
    else if(gps_msg->status.status == 1) 
    {
        
        ROS_DEBUG_THROTTLE(60,"GPS  float.");
        return;
    }
    else if(gps_msg->status.status == 2) 
    {
        //ROS_INFO("GPS2  FIX.");
    }
    else
    {
    }
} 
                              
void imu_link_restart_Callback(const std_msgs::Bool& msg) 
{
    imu_link_restart_flag = msg.data;
}
void car_speedCallback(const std_msgs::Float32::ConstPtr& msg) 
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_gps_heading_angle_link_new");
    ros::NodeHandle n;
    
    std_msgs::Float32 imu_offset_angle;
    std_msgs::Float32 imu_offset_angle_degree;
    std_msgs::Float32 imu_heading_angle_msg;
    unsigned long    cnt = 0;
    double  sum_offset = 0.0;
    double  avg_gps_heading_angle_radian = 0.0;
    double  avg_imu_heading_angle_radian = 0.0;
    double  corrected_imu_angle_radian   = 0.0;
    double  yaw_error                    = 0.0;
    double  auto_imu_offset_radian       = 0.0;
    /* topci ID */
    
    std::string imu1_topic                             = "/handsfree/imu/data";
    std::string imu1_yaw_degree_topic                  = "/handsfree/imu/yaw_degree";
    std::string imu1_yaw_radian_topic                  = "/handsfree/imu/yaw_radian";   
        
    std::string imu2_topic                             = "/robor/imu/data";
    std::string imu2_yaw_degree_topic                  = "/robor/imu/yaw_degree";
    std::string imu2_yaw_radian_topic                  = "/robor/imu/yaw_radian";
                
    std::string imu_heading_angle_radian_topic         = "/imu/heading_angle_radian";
    std::string imu_heading_angle_degree_topic         = "/imu/heading_angle_degree";
    std::string imu_heading_angle_offset_degree_topic  = "/imu/heading_angle_offset_degree";
        
        
    std::string gps1_utm_topic                         = "/gps/utm_pos1";
    std::string gps2_utm_topic                         = "/gps/utm_pos2";
    std::string gps_heading_angle_topic                = "/gps/heading_angle";
    std::string car_speed_topic                        = "/Car_Control_cmd/Speed_Float32";
    std::string imu_correction_enable_topic            = "/flag/imu_auto_correction";
    
    
    std::string imu_correction_restart_topic           = "/set/imu_correction_restart";
        
    ros::param::get("~imu1_topic", imu1_topic);
    ros::param::get("~imu1_yaw_degree_topic", imu1_yaw_degree_topic );
    ros::param::get("~imu1_yaw_radian_topic", imu1_yaw_radian_topic);
    
    
    ros::param::get("~imu2_topic", imu2_topic);
    ros::param::get("~imu2_yaw_degree_topic", imu2_yaw_degree_topic );
    ros::param::get("~imu2_yaw_radian_topic", imu2_yaw_radian_topic);
    
    ros::param::get("~choose_imu_no", choose_imu_no);
    
    
    ros::param::get("~imu_heading_angle_radian_topic", imu_heading_angle_radian_topic);
    ros::param::get("~imu_heading_angle_degree_topic", imu_heading_angle_degree_topic);
    ros::param::get("~imu_heading_angle_offset_degree_topic", imu_heading_angle_offset_degree_topic);
    
        
    
    ros::param::get("~gps1_utm_topic", gps1_utm_topic);
    ros::param::get("~gps2_utm_topic", gps2_utm_topic);
    ros::param::get("~gps_heading_angle",gps_heading_angle_topic);  
    
    ros::param::get("~car_speed_topic",car_speed_topic);
    
    ros::param::get("~imu_yaw_offset_degree",imu_yaw_offset_degree);
    ros::param::get("~auto_imu_offset_flag", auto_imu_offset_flag);
        
     
    ros::Subscriber sub_utm1                      = n.subscribe(gps1_utm_topic,1, &gps_utm_pose1Callback);
    ros::Subscriber sub_utm2                      = n.subscribe(gps2_utm_topic,1, &gps_utm_pose2Callback);
    ros::Subscriber sub_fix1                      = n.subscribe("/gps1/fix",1,&gps1_fix_check_Callback);  // front gps 
    ros::Subscriber sub_fix2                      = n.subscribe("/gps2/fix",1,&gps2_fix_check_Callback);  // front gps  
    ros::Subscriber sub_gps_heading               = n.subscribe(gps_heading_angle_topic,1,&GPSHeadingAngleCallback);
    ros::Subscriber sub_enable_imu_correction     = n.subscribe(imu_correction_enable_topic, 1,&imu_correction_enable_Callback);
    ros::Subscriber sub_gps_imu_link_restart     = n.subscribe(imu_correction_restart_topic, 1,&imu_link_restart_Callback);
    
    ros::Subscriber sub_imu1                      = n.subscribe(imu1_topic,1,&imu1Callback);
    ros::Subscriber sub_imu1_yaw_radian           = n.subscribe(imu1_yaw_radian_topic,1,&imu1yawradianCallback);
            
    
    ros::Subscriber sub_imu2                      = n.subscribe(imu2_topic,1,&imu1Callback);
    ros::Subscriber sub_imu2_yaw_radian           = n.subscribe(imu2_yaw_radian_topic,1,&imu2yawradianCallback);
    
    
    ros::Subscriber sub_car_speed                  = n.subscribe(car_speed_topic,1, &car_speedCallback);
        
    ros::Publisher imu_offest_angle_degree_pub    = n.advertise<std_msgs::Float32>(imu_heading_angle_offset_degree_topic, 1);
    ros::Publisher imu_heading_angle_radian_pub   = n.advertise<std_msgs::Float32>(imu_heading_angle_radian_topic, 1);
    
    ros::Rate loop_rate(50);  // 10
    
	while (ros::ok())
	{
        
		boost::shared_ptr<sensor_msgs::NavSatFix const> shared_gps_fix1_topic, shared_gps_fix2_topic;

		//sensor_msgs::NavSatFix topic_gps_fix1, topic_gps_fix2;
		shared_gps_fix1_topic  = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps1/fix",n,ros::Duration(0.01) );
		shared_gps_fix2_topic  = ros::topic::waitForMessage<sensor_msgs::NavSatFix>("/gps2/fix",n,ros::Duration(0.01) );

		boost::shared_ptr<sensor_msgs::Imu const> shared_imu1_topic, shared_imu2_topic;

		if(shared_gps_fix1_topic == NULL)
		{
			 ROS_WARN("No gps1 topic");  
		}
		else
		{
			//topic_gps_fix1 = *shared_gps_fix1_topic;
		}

		if(shared_gps_fix2_topic == NULL)
		{
			 ROS_WARN("No gps2 topic");  
		}
		else
		{
			//topic_gps_fix2 = *shared_gps_fix2_topic;
		}
        
        
        //////////////////////////// imu 선택  /////////////////////////////////////////
        if(choose_imu_no == 1)  imu_heading_angle_radian  = imu1_heading_angle_radian;
		if(choose_imu_no == 2)  imu_heading_angle_radian  = imu2_heading_angle_radian;
                
        if(auto_imu_offset_flag == false) // 수동 보정 모드 : 미리 세팅된 값으로 보정함 
        {
            printf("Manual IMU Offset Correction Mode! \n");
            corrected_imu_angle_radian = imu_heading_angle_radian + DEG2RAD(imu_yaw_offset_degree);  //수동 보정 모드 : 값은 launch에서 주어딤
                       
        }
        
        else // 자동 보정 모드
        {
			printf("Auto IMU Offset Correction Mode! \n");
            if( (shared_gps_fix1_topic != NULL) && (shared_gps_fix2_topic != NULL) )
			{
        
				if( (gps_fix1 == 2) &&(gps_fix2 == 2) )         
				{
					if( enable_imu_correction == true )  // 자동 보정을 사용함 
					{
						 
						imu_offset            = gps_heading_angle_radian - imu_heading_angle_radian;   // gps와 imu의 차이값을 계산 
						
						imu_yaw_offset_degree = RAD2DEG(imu_offset);
						
						imu_offset_angle_degree.data = RAD2DEG(imu_offset);                              // radian 값을 degree 로 변경  
						imu_offest_angle_degree_pub.publish(imu_offset_angle_degree); 
						
						printf("\n");
						printf("gps heading : %6.3lf | imu heading : %6.3lf | imu offset : %6.3lf \n",RAD2DEG(gps_heading_angle_radian),RAD2DEG(imu_heading_angle_radian), imu_offset_angle_degree.data);
						
						yaw_error = corrected_imu_angle_radian - imu_heading_angle_radian;
						
						if( fabs(yaw_error) > DEG2RAD(3) ) // 3도 이상 오차가 나면 보정
						{
							corrected_imu_angle_radian = imu_heading_angle_radian + imu_offset;  //imu 보정함
						}
						
						
					}
					else
					{ 
						printf("Disable IMU Auto Offset Correction Mode! \n");
						corrected_imu_angle_radian = imu_heading_angle_radian + imu_offset;  //imu 기존 보정 값을 사용
						
					}	
					
					imu_heading_angle_msg.data = corrected_imu_angle_radian;
					if(imu_heading_angle_msg.data >  M_PI)   imu_heading_angle_msg.data -= 2*M_PI;
					if(imu_heading_angle_msg.data < -M_PI)   imu_heading_angle_msg.data += 2*M_PI;
									
					imu_heading_angle_radian_pub.publish(imu_heading_angle_msg);
					printf("GPS heading angle : auto corrected_imu_angle  = [%5.2lf : %5.2lf]\n\n", RAD2DEG(gps_heading_angle_radian), RAD2DEG(imu_heading_angle_msg.data) );                          
					
				}//if( (gps_fix1 == 2) &&(gps_fix2 == 2) )   
				
				else // GPS가 fix가 풀렸을 때
				{
					ROS_ERROR("GPS no fix!");
					
					corrected_imu_angle_radian = imu_heading_angle_radian + imu_offset;  //imu 기존 보정 값을 사용					
					imu_heading_angle_msg.data =  corrected_imu_angle_radian;
					if(imu_heading_angle_msg.data >  M_PI)   imu_heading_angle_msg.data -= 2*M_PI;
					if(imu_heading_angle_msg.data < -M_PI)   imu_heading_angle_msg.data += 2*M_PI;
					imu_heading_angle_radian_pub.publish(imu_heading_angle_msg);                
					
					printf("no fix corrected_imu_angle : %6.3lf\n\n", RAD2DEG(corrected_imu_angle_radian) ); 		
				}
			}//if( (shared_gps_fix1_topic != NULL) && (shared_gps_fix2_topic != NULL) )
			else
			{
				ROS_ERROR("GPS NO GPS!");
				corrected_imu_angle_radian = imu_heading_angle_radian + imu_offset;  //imu 기존 보정 값을 사용					
				imu_heading_angle_msg.data =  corrected_imu_angle_radian;
				if(imu_heading_angle_msg.data >  M_PI)   imu_heading_angle_msg.data -= 2*M_PI;
				if(imu_heading_angle_msg.data < -M_PI)   imu_heading_angle_msg.data += 2*M_PI;
				imu_heading_angle_radian_pub.publish(imu_heading_angle_msg);                
				
				printf("NO GPS corrected_imu_angle : %6.3lf\n\n", RAD2DEG(corrected_imu_angle_radian) );
			}	        
		}          
		loop_rate.sleep();
		ros::spinOnce();
        
    } //end of while
    return 0;
    
}
     
