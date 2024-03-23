#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "math.h"
#include "move_waypoint/Target_waypoint_line.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
#define MAX_WAYPOINT_NUM 300

enum type
{
    RESET = -1,
    STOP,
    RUN,
    ARRIVAL
};

#define NEW_WAYPOINT_TOR_XY     0.01
#define NEW_WAYPOINT_TOR_THETA  5.0          

typedef struct
{
    double a;
    double b;
    double c;
    double d;
} Line_Equation;

typedef struct
{
    double x;
    double y;
    double theta;
} Vector_2D;

geometry_msgs::Pose2D target_pose2D;
geometry_msgs::Pose2D start_pose2D;

double tx, ty, tz, roll, pitch, yaw;

void Pose_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    tx = msg->pose.pose.position.x;
    ty = msg->pose.pose.position.y;
    tz = msg->pose.pose.position.z;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);
}

bool flag_mew_waypoint       = false;
bool flag_new_start_id       = false;
bool flag_new_finish_id      = false;
int flag_waypoint_control    = STOP;
int no_waypoints             =-1;
int status_waypoint_move     = 0;
int waypoint_finish_id_old   = 0;
int waypoint_start_id_old    =-1;
int waypoint_move            = 0;
int waypoint_start_id        =-1;
int waypoint_finish_id       = 0;

std::string waypoint_file = "/home/amap/sim_catkin_ws/src/waypoint_control_system/data/waypoint1.txt";

void waypoint_move_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    waypoint_move = msg->data;
}

void waypoint_start_id_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    waypoint_start_id = msg->data;
    if(waypoint_start_id != waypoint_start_id_old)
    {
      flag_new_start_id = true;
      waypoint_start_id_old = waypoint_start_id;
      printf("new start id id recevied\n");
   }
   
   else
   {
      flag_new_start_id = false;
   }
}

void waypoint_finish_id_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    waypoint_finish_id = msg->data;
    if(waypoint_start_id != waypoint_start_id_old)
    {
      flag_new_start_id = true;
      waypoint_start_id_old = waypoint_start_id;
      printf("new finish idd is received!\n");
   }
   else
   {
      flag_new_finish_id = false;
    }
}

move_waypoint::Target_waypoint_line waypoint[MAX_WAYPOINT_NUM];

bool read_waypoint_file(std::string File_Name)
{
    bool success = false;
    int result = -10;
    
    FILE *fp;
    fp = fopen(File_Name.c_str(),"r");
    int no_waypoints = -1; // 웨이포인트 개수 초기화
   
    if(fp == NULL) 
    {
        ROS_ERROR("Waypoint File Does not Exist!!\n");
        return success; 
    }
    else
    {
        printf("File Open Success\n");
        do
        {
            ++no_waypoints; 
            result = fscanf(fp,"%lf %lf %lf", &waypoint[no_waypoints].waypoint_target_pose2d.x, &waypoint[no_waypoints].waypoint_target_pose2d.y, &waypoint[no_waypoints].waypoint_target_pose2d.theta);
            printf("waypoint_target.x[%.2lf]   waypoint_target.y[%.2lf]   waypoint_target.theta[%.2lf]\n", waypoint[no_waypoints].waypoint_target_pose2d.x, waypoint[no_waypoints].waypoint_target_pose2d.y, waypoint[no_waypoints].waypoint_target_pose2d.theta);
        } while(result != EOF);
      
        waypoint[no_waypoints].waypoint_start_pose2d.x     = 0.0;
        waypoint[no_waypoints].waypoint_start_pose2d.y     = 0.0;
        waypoint[no_waypoints].waypoint_start_pose2d.theta = 0.0;
      
        for(int i = 1; i < no_waypoints; i++)
        {
            waypoint[i].waypoint_start_pose2d.x     = waypoint[i - 1].waypoint_target_pose2d.x;
            waypoint[i].waypoint_start_pose2d.y     = waypoint[i - 1].waypoint_target_pose2d.y;
            waypoint[i].waypoint_start_pose2d.theta = waypoint[i - 1].waypoint_target_pose2d.theta;
            printf("waypoint_start.x[%.2lf]    waypoint_start.y[%.2lf]    waypoint_start.theta[%.2lf]\n",waypoint[i].waypoint_start_pose2d.x,waypoint[i].waypoint_start_pose2d.y,waypoint[i].waypoint_start_pose2d.theta);
        }
        waypoint[no_waypoints].waypoint_start_pose2d.x     = 0.0;
        waypoint[no_waypoints].waypoint_start_pose2d.y     = 0.0;
        waypoint[no_waypoints].waypoint_start_pose2d.theta = 0.0;
        for(int i = 0; i < no_waypoints; i++)
        {
            waypoint[i].waypoint_start_pose2d.x     = waypoint[i - 1].waypoint_target_pose2d.x;
            waypoint[i].waypoint_start_pose2d.y     = waypoint[i - 1].waypoint_target_pose2d.y;
            waypoint[i].waypoint_start_pose2d.theta = waypoint[i - 1].waypoint_target_pose2d.theta;
            printf("waypoint_start.x[%.2lf]    waypoint_start.y[%.2lf]    waypoint_start.theta[%.2lf]\n",waypoint[i].waypoint_start_pose2d.x,waypoint[i].waypoint_start_pose2d.y,waypoint[i].waypoint_start_pose2d.theta);
        }
    }
    fclose(fp);

    success = true;    

    return success; 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_control_system_node");
    ros::NodeHandle n;

    int count = 0;
    int start_waypoint_id = 0;
    int finish_waypoint_id = 0; 

    std::string poseupdate_topic                     = "/poseupdate";
    std::string type_cmd_vel_topic                   = "/type_cmd_vel";
    std::string waypoint_file                        = "/home/amap/sim_catkin_ws/src/waypoint_control_system/data/waypoint1.txt";
    std::string waypoint_move_topic                  = "/status/waypoint_move";
    std::string target_waypoint_move_topic           = "/wp/target_waypoint_line";  //int8
    std::string waypoint_run_command_topic           = "/flag/amr_run"; //bool
    std::string waypoint_start_id_topic              = "/start_id_no"; //int8
    std::string waypoint_finish_id_topic             = "/finish_id_no"; //int8

    ros::param::get("~poseupdate_topic",             poseupdate_topic);
    ros::param::get("~type_cmd_vel_topic",           type_cmd_vel_topic);
    ros::param::get("~waypoint_file"   ,             waypoint_file);
    ros::param::get("~waypoint_move_topic",          waypoint_move_topic);
    ros::param::get("~target_waypoint_move_topic",   target_waypoint_move_topic);
    ros::param::get("~waypoint_run_command_topic",   waypoint_run_command_topic);
    ros::param::get("~waypoint_start_id_topic",      waypoint_start_id_topic);
    ros::param::get("~waypoint_finish_id_topic",     waypoint_finish_id_topic);
    
    ros::Subscriber sub_poseupdate                      = n.subscribe(poseupdate_topic,10, Pose_Callback); 
    ros::Subscriber sub_waypoint_move_topic             = n.subscribe(waypoint_move_topic, 10, waypoint_move_Callback);
    ros::Subscriber sub_waypoint_start_id_topic         = n.subscribe(waypoint_start_id_topic,10, waypoint_start_id_Callback); 
    ros::Subscriber sub_waypoint_finish_id_topic        = n.subscribe(waypoint_finish_id_topic,10, waypoint_finish_id_Callback); 
    ros::Publisher pub_type_cmd_vel                     = n.advertise<std_msgs::Int8>(type_cmd_vel_topic, 1);
    ros::Publisher pub_waypoint_run_command             = n.advertise<std_msgs::Int8>(waypoint_run_command_topic, 1);
    ros::Publisher target_waypoint_move_pub             = n.advertise<move_waypoint::Target_waypoint_line>(target_waypoint_move_topic, 1);
    
    read_waypoint_file(waypoint_file);
    double current_waypoint_id = 0.0;

    ros::Rate loop_rate(100.0);
    
    while (ros::ok())
    { 
      if(flag_new_start_id == true)
      {
         current_waypoint_id = waypoint_start_id;
         flag_new_start_id = false;
      }
      
      if(count <= 20)
      {
         move_waypoint::Target_waypoint_line waypoint_line_msg = waypoint[(int)current_waypoint_id];
         target_waypoint_move_pub.publish(waypoint_line_msg);
      }
        
        
        if(flag_waypoint_control == RUN)
        {
            current_waypoint_id++;
            printf("------------------------------------");
            printf("---------------RUN------------------");
            printf("------------------------------------");
        }
        else if(flag_waypoint_control == ARRIVAL)
        {
            current_waypoint_id++;
            printf("------------------------------------");
            printf("--------------ARRIVAL---------------");
            printf("------------------------------------");
            ros::Duration(1.0).sleep(); // 1초 딜레이
            count = 0;
        }
        else if(flag_waypoint_control == STOP)
        {
            current_waypoint_id++;
            printf("------------------------------------");
            printf("---------------STOP-----------------");
            printf("------------------------------------");
            
        }
        else
        {
            current_waypoint_id++;
            printf("------------------------------------");
            printf("---------------RESET----------------");
            printf("------------------------------------");
        }
        if(current_waypoint_id >= no_waypoints - 1)
        {
            current_waypoint_id = no_waypoints - 1;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        if(count>1000) count =1000;
    }

    return 0;
}
