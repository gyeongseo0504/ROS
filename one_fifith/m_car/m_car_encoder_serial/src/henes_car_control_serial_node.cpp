#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Pose2D.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <string>

#define LOOP_RATE 50
#define Right_MAX -24
#define Left_MAX   25

// 전역 변수
static int can_fd = -1;

// 명령 값
int steering_angle = 0, steering_angle_cmd = 0;
float motor_speed = 0.0, motor_speed_cmd = 0.0;

// 피드백 값
int current_rpm_fb = 0;
int current_steer_fb = 90;

// 오도메트리 계산용
double yaw = 0.0, yaw_old = 0.0;
geometry_msgs::Pose2D my_utm_pose1;
bool odom_reset_flag = false;

// 주행 파라미터
double m_acceleration = 0.5, m_deceleration = 0.5;
double s_acceleration = 1.0, s_deceleration = 1.0;

// CAN 초기화
int init_can_port(const std::string& ifname = "can0") {
    struct ifreq ifr;
    struct sockaddr_can addr;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;

    strcpy(ifr.ifr_name, ifname.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -2;
    return s;
}

// CAN 송신
void send_can_data() {
    struct can_frame frame;
    frame.can_id  = 0x01;
    frame.can_dlc = 8;

    if(motor_speed_cmd > 0)      frame.data[0] = 0x82;
    else if(motor_speed_cmd < 0) frame.data[0] = 0x02;
    else                         frame.data[0] = 0x00;

    uint16_t rpm_val = static_cast<uint16_t>(fabs(motor_speed_cmd));
    frame.data[1] = rpm_val & 0xFF;
    frame.data[2] = (rpm_val >> 8) & 0xFF;

    frame.data[3] = 0xAA; 
    frame.data[4] = static_cast<uint8_t>(steering_angle_cmd + 90);

    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    write(can_fd, &frame, sizeof(frame));
}

// 오도메트리 계산
struct OdomCaculateData {
    float distance_ratio= 348.0;
    float position_x=0.0;
    float position_y=0.0;
    float oriention=0.0;
} myOdom;

void odometry_cal() {
    double delta_yaw = yaw - yaw_old;
    double base_link_delta_x = (double)current_rpm_fb / myOdom.distance_ratio;
    double base_link_delta_y = 0.0;

    double odom_delta_x = base_link_delta_x * cos(yaw_old) - base_link_delta_y * sin(yaw_old);
    double odom_delta_y = base_link_delta_x * sin(yaw_old) + base_link_delta_y * cos(yaw_old);

    myOdom.position_x += odom_delta_x;
    myOdom.position_y += odom_delta_y;
    myOdom.oriention = yaw;
    yaw_old = yaw;
}

// CAN 수신 스레드
void* read_can_thread(void* arg) {
    struct can_frame frame;
    while (ros::ok()) {
        int nbytes = read(can_fd, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && frame.can_id == 0x302) {
            current_rpm_fb = frame.data[1] | (frame.data[2] << 8);
            current_steer_fb = frame.data[4];
            ROS_INFO("RX 0x302 Feedback - RPM: %d, Steer: %d", current_rpm_fb, current_steer_fb);
        }
    }
    return NULL;
}

// ROS 콜백
void imu_heading_angleCallback(const std_msgs::Float32& msg) { yaw = msg.data; }
void utm_odom_Callback(const nav_msgs::Odometry& msg) {
    my_utm_pose1.x = msg.pose.pose.position.x;
    my_utm_pose1.y = msg.pose.pose.position.y;
}
void CarControlCallback(const geometry_msgs::Twist& msg) {
    steering_angle_cmd = std::max(std::min((int)msg.angular.z, Left_MAX), Right_MAX);
    motor_speed_cmd = msg.linear.x;
}
void CarSteerControlCallback(const std_msgs::Int16& angle) {
    steering_angle_cmd = std::max(std::min((int)angle.data, Left_MAX), Right_MAX);
}
void CarSpeedControlCallback(const std_msgs::Float32& speed) {
    motor_speed_cmd = speed.data;
    if(motor_speed_cmd > 16000) motor_speed_cmd = 16000;
    if(motor_speed_cmd < -16000) motor_speed_cmd = -16000;
}
void odom_reset_flag_Callback(const std_msgs::Bool& flag) { odom_reset_flag = flag.data; }

// 속도/조향 profile
void robot_speed_profile_control() {
    if(motor_speed > motor_speed_cmd) {
        motor_speed -= (m_deceleration / LOOP_RATE) * 20;
        if(motor_speed < motor_speed_cmd) motor_speed = motor_speed_cmd;
    } else if(motor_speed < motor_speed_cmd) {
        motor_speed += (m_acceleration / LOOP_RATE) * 20;
        if(motor_speed > motor_speed_cmd) motor_speed = motor_speed_cmd;
    }
}
void robot_steer_profile_control() {
    if(steering_angle > steering_angle_cmd) {
        steering_angle -= s_deceleration / LOOP_RATE;
        if(steering_angle < steering_angle_cmd) steering_angle = steering_angle_cmd;
    } else if(steering_angle < steering_angle_cmd) {
        steering_angle += s_acceleration / LOOP_RATE;
        if(steering_angle > steering_angle_cmd) steering_angle = steering_angle_cmd;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "henes_car_control_can");
    ros::NodeHandle n;

    ros::param::get("~m_acceleration", m_acceleration);
    ros::param::get("~m_deceleration", m_deceleration);
    ros::param::get("~s_acceleration", s_acceleration);
    ros::param::get("~s_deceleration", s_deceleration);

    can_fd = init_can_port("can0");
    if(can_fd < 0) {
        ROS_ERROR("CAN port open failed");
        return -1;
    }
    pthread_t tid;
    pthread_create(&tid, NULL, read_can_thread, NULL);

    ros::Subscriber sub1 = n.subscribe("/cmd_vel", 1, CarControlCallback);
    ros::Subscriber sub2 = n.subscribe("/Car_Control_Cmd/steerAngle_Int16", 1, CarSteerControlCallback);
    ros::Subscriber sub3 = n.subscribe("/Car_Control_Cmd/speed_Float32", 1, CarSpeedControlCallback);
    ros::Subscriber sub4 = n.subscribe("/flag/odom_reset", 1, odom_reset_flag_Callback);
    ros::Subscriber sub5 = n.subscribe("/odom/utm", 1, utm_odom_Callback);
    ros::Subscriber sub6 = n.subscribe("/imu/heading_angle_radian", 10, imu_heading_angleCallback);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom/car", 20);
    ros::Publisher rpm_pub = n.advertise<std_msgs::Int32>("/current_rpm", 1);
    ros::Publisher steer_pub = n.advertise<std_msgs::Int16>("/current_steer_angle", 1);

    ros::Rate rate(LOOP_RATE);

    while(ros::ok()) {
        robot_speed_profile_control();
        robot_steer_profile_control();
        send_can_data();

        odometry_cal();
        if(odom_reset_flag) {
            myOdom.position_x = my_utm_pose1.x;
            myOdom.position_y = my_utm_pose1.y;
            odom_reset_flag = false;
        }

        // 피드백 퍼블리시
        std_msgs::Int32 rpm_msg; rpm_msg.data = current_rpm_fb; rpm_pub.publish(rpm_msg);
        std_msgs::Int16 steer_msg; steer_msg.data = current_steer_fb; steer_pub.publish(steer_msg);

        // 오도메트리 퍼블리시
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(myOdom.oriention);
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = myOdom.position_x;
        odom.pose.pose.position.y = myOdom.position_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom_pub.publish(odom);

        ros::spinOnce();
        rate.sleep();
    }

    close(can_fd);
    return 0;
}
