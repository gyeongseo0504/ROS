#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "math.h"

#define TSL1401CL_SIZE 320 
#define DEG2RAD(x) (M_PI / 180.0) * x
#define RAD2DEG(x) (180.0 / M_PI) * x
int mission_flag = 0;
int stop = 1000;
int Finish = 1001;
//////////////////////////////////////////////////////////////////////// line
double tsl1401cl_data[TSL1401CL_SIZE];
int LineSensor_threshold_Data[TSL1401CL_SIZE];

double Line_Center = 147;
double OFFSET = 0;

void threshold(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    double THRESHOLD = 0.01;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        tsl1401cl_data[i] = msg->data[i];
    }

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        if (tsl1401cl_data[i] > THRESHOLD)
        {
            LineSensor_threshold_Data[i] = 180;  // 라인이 검출되는 곳은 180 출력
        }
        else
        {
            LineSensor_threshold_Data[i] = 0;  // 아닌 곳은 0 출력
        }
    }
    printf("Threshold Data: \n");

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        printf("%d ", LineSensor_threshold_Data[i]);
    }
    printf("\n");
}

int find_line_center()
{
    int centroid = 0;
    int mass_sum = 0;

    for (int i = 0; i < TSL1401CL_SIZE; i++)
    {
        mass_sum += LineSensor_threshold_Data[i];
        centroid += LineSensor_threshold_Data[i] * i;
    }

    if (mass_sum == 0)// mass_sum이 0이라는건 라인이 안잡힌다는것
    {
        return stop;
    }
    else if(mass_sum == 180 * TSL1401CL_SIZE)//트랙이 아닌곳은 카메라에서 전부 인식하는것으로 보임
    {                                        //모두 180이 인식되면 트랙 밖이 카메라에 보인다는것
		return Finish;
	}
    
    centroid = centroid / mass_sum;
    printf("Line Centroid: %d\n", centroid);
    return centroid;
}

geometry_msgs::Twist lane_control(double centroid, double kp, double ki, double kd)
{
    geometry_msgs::Twist cmd_vel;

    double error = 0.0;
    double error_d = 0.0;
    double error_old = 0.0;
    double Steering_Angle = 0.0;

    if (centroid == 0.0)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        error = Line_Center - centroid + OFFSET;
        error_d = error - error_old;
        Steering_Angle = kp * error + kd * error_d + ki * 0;

        cmd_vel.linear.x = 0.4;
        cmd_vel.angular.z = Steering_Angle / 130;

        error_old = error;
    }

    return cmd_vel;
}
//////////////////////////////////////////////////////////////////////// line
//////////////////////////////////////////////////////////////////////// yaw
double roll, pitch, yaw;
double error_old = 0.0;

double normalizeYaw(double yaw_deg)
{
    if (yaw_deg > 360)
    {
        yaw_deg = yaw_deg - 360;//각이 360보다 커지면 -360
    }
    else if (yaw_deg < 0)
    {
        yaw_deg = yaw_deg + 360;//각이 음수가 나오면 +360
    }

    return yaw_deg;
}

geometry_msgs::Twist PID_yaw_control(double Kp, double Ki, double Kd, double target_yaw_degree)
{
    geometry_msgs::Twist cmd_vel;

    double yaw_deg = RAD2DEG(yaw);
    yaw_deg = normalizeYaw(yaw_deg);

    double error = target_yaw_degree - yaw_deg;

    if (error > 180)
    {
        error = error - 360;//180보다 크다면, error에서 360을 빼서 반대 방향으로 회전
    }
    else if (error < -180)
    {
        error = error + 360;//-180보다 작다면, error에 360을 더해 반대 방향으로 회전
    }

    double error_sum = 0.0;
    double error_d = error - error_old;
    error_sum += error;

    double Steering_Angle = Kp * error + Ki * error_sum + Kd * error_d;

    cmd_vel.linear.x = 0.4;
    cmd_vel.angular.z = Steering_Angle;

    if (fabs(error) < 0.5)
    {
        cmd_vel.linear.x = 0.0;
        //cmd_vel.angular.z = 0.0;
        mission_flag++;
    }

    error_old = error;

    return cmd_vel;
}

void imu1Callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    double yaw_deg = normalizeYaw(RAD2DEG(yaw));
    printf("%f\n", yaw_deg);
}
//////////////////////////////////////////////////////////////////////// yaw
//////////////////////////////////////////////////////////////////////// wall
double front_sonar = 0.0;
double left_sonar = 0.0;
double right_sonar = 0.0;

void FrontSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  front_sonar = msg->range;
  printf("Front Sonar: %.2f    ", front_sonar);
}

void LeftSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  left_sonar = msg->range;
  printf("Left Sonar:  %.2f    ", left_sonar);
}

void RightSonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  right_sonar = msg->range;
  printf("Right Sonar: %.2f\n", right_sonar);
}

geometry_msgs::Twist WallFollowing(double kp, double ki, double kd)//geometry_msgs::Twist 타입의 메세지 반환하는 함수 
{
  geometry_msgs::Twist cmd_vel;
  double error = left_sonar - right_sonar;
  double error_old = 0.0;
  double error_d = error - error_old;  
  double error_sum = 0.0;
  error_sum += error;  
  
  double steering_control = kp * error + ki * error_sum + kd * error_d;

  if (front_sonar < 0.9)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  else
  {
    cmd_vel.linear.x = 0.4;
    cmd_vel.angular.z = steering_control;
  }

  error_old = error;

  return cmd_vel;
}
//////////////////////////////////////////////////////////////////////// wall
int main(int argc, char **argv)
{
    int count = 0;

    ros::init(argc, argv, "Pioneer");
    ros::NodeHandle n;

    ros::Subscriber tsl1401cl_sub = n.subscribe("/tsl1401cl", 10, threshold);
    ros::Subscriber yaw_control_sub = n.subscribe("/imu", 1000, imu1Callback);
    ros::Subscriber front_sonar_sub = n.subscribe("range_front", 1000, FrontSonarCallback);
    ros::Subscriber left_sonar_sub = n.subscribe("range_front_left", 1000, LeftSonarCallback);
    ros::Subscriber right_sonar_sub = n.subscribe("range_front_right", 1000, RightSonarCallback);
  
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/ackermann_steering_controller/cmd_vel", 1000);
    geometry_msgs::Twist cmd_vel;
    
    double kp_line = 0.15;
    double ki_line = 0.0;
    double kd_line = 0.1;
    
    double kp_yaw = 0.02;
    double kd_yaw = 0.3;
    double ki_yaw = 0.0;
    
    double kp_yaw2 = 0.03;
    double kd_yaw2 = 0.5;
    double ki_yaw2 = 0.0;
    
    double kp_wall = 0.35;
    double kd_wall = 0.35;
    double ki_wall = 0.0;
    
    double target_yaw_degree = 0.0;//원하는 각도 입력

    ros::Rate loop_rate(30.0);
    while (ros::ok())
    {
        switch (mission_flag)
        {
        case 0:
			if(find_line_center() == stop)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
			}
			else
			{
				mission_flag++;

			}
            break;
        case 1:
			if(find_line_center() != stop && find_line_center() != Finish)
			{
				cmd_vel = lane_control(find_line_center(), kp_line, ki_line, kd_line);
			}
			else
			{
				mission_flag++;
			}
            break;
        case 2:
			if(front_sonar > 1.0)
			{
				cmd_vel.linear.x = 0.4;
				cmd_vel.angular.z = 0.0;
			}
			else
			{
				target_yaw_degree = 260.0;//원하는 각도 입력
				mission_flag++;
			}
            break;
        case 3:
			cmd_vel = PID_yaw_control(kp_yaw, ki_yaw, kd_yaw, target_yaw_degree);
            break;
        case 4:
			cmd_vel = WallFollowing(kp_wall, ki_wall, kd_wall);
			if(front_sonar < 0.9)
			{
				target_yaw_degree = 175.0;//원하는 각도 입력
				mission_flag++;
			}
            break;
        case 5:
			cmd_vel = PID_yaw_control(kp_yaw2, ki_yaw2, kd_yaw2, target_yaw_degree);
			break;
		case 6:
			cmd_vel = lane_control(find_line_center(), kp_line, ki_line, kd_line);
			if(find_line_center() == Finish)
			{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
			}
			break;
			
        }
		cmd_vel_pub.publish(cmd_vel);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
