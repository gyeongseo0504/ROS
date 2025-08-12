#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <cmath>

template<typename T>
T clamp(T v, T lo, T hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 조향 관련 변수
double yolo_goal_angle = 0.0;     // goal_angle에서 들어온 값
bool use_yolo_goal_angle = false; // 최근 YOLO 조향 각도 갱신 여부

// 기존 상위 제어도 남겨둘 경우 아래 변수도 같이 씀
int steering = 90;   // 69~111 (직진 90)
float speed = 0.0;   // -16000 ~ 16000

// goal_angle 콜백
void yoloGoalAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    yolo_goal_angle = msg->data;
    use_yolo_goal_angle = true;
}

// 기존 경로제어 신호도 남겨두고 싶으면 같이 사용
void steeringCallback(const std_msgs::Int16::ConstPtr& msg) {
    steering = msg->data;
}
void speedCallback(const std_msgs::Float32::ConstPtr& msg) {
    speed = msg->data;
}

// goal_angle을 CAN 조향값(69~111, 90이 중심)으로 맵핑
uint8_t convertGoalAngleToSteer(double angle) {
    // angle: -값(좌회전), 0(직진), +값(우회전)
    // 실차 기준 -21~21 → 69~111, 0이면 90
    int steer_val = 90 + static_cast<int>(angle); // 필요시 비율 보정도 곱할 것!
    steer_val = clamp(steer_val, 69, 111);
    return static_cast<uint8_t>(steer_val);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_tr_node");
    ros::NodeHandle nh;

    ros::Subscriber yolo_goal_angle_sub = nh.subscribe("/goal_angle", 1, yoloGoalAngleCallback);
    // 필요시 기존 steering, speed 신호도 구독
    ros::Subscriber steer_sub = nh.subscribe("/Car_Control_Cmd/xte_steerAngle_Int16", 1, steeringCallback);
    ros::Subscriber speed_sub = nh.subscribe("/Car_Control_Cmd/speed_Float32", 1, speedCallback);

    int can_sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    const char* ifname = "can0";

    if ((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("CAN socket");
        return 1;
    }
    std::strcpy(ifr.ifr_name, ifname);
    ioctl(can_sock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("CAN bind");
        return 1;
    }

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        struct can_frame frame;
        frame.can_id = 0x01;
        frame.can_dlc = 8;

        // 명령/방향 처리
        uint8_t cmd = 0x00;
        int rpm_val = 0;
        float input_speed = speed;
        if (input_speed > 0.5) {
		cmd = 0x82;
		rpm_val = static_cast<int>(clamp(input_speed, 0.0f, 16000.0f));
		} else if (input_speed < -0.5) {
		cmd = 0x02;
		rpm_val = static_cast<int>(clamp(-input_speed, 0.0f, 16000.0f));
		} else {
			cmd = 0x00;
		rpm_val = 0;
		}
        uint8_t rpm_L = rpm_val & 0xFF;
        uint8_t steer_val = 90; // 기본값

        // YOLO에서 goal_angle이 들어오면 우선 적용
        if (use_yolo_goal_angle) {
            steer_val = convertGoalAngleToSteer(yolo_goal_angle);
            use_yolo_goal_angle = false; // 1회성 (갱신될 때만 반영)
        } else {
            steer_val = static_cast<uint8_t>(clamp(steering, 69, 111));
        }

        frame.data[0] = cmd;
        frame.data[1] = rpm_L;
        frame.data[2] = 0x01;
        frame.data[3] = 0xAA;
        frame.data[4] = steer_val;
        frame.data[5] = 0x00;
        frame.data[6] = 0x00;
        frame.data[7] = 0x00;

        printf("[CAN][goal_angle:%.2f steering:%d] speed: %.2f, rpm_L:0x%02X, cmd:0x%02X, steer:%d(0x%02X) [",
            yolo_goal_angle, steering, speed, rpm_L, cmd, steer_val, steer_val);
        for (int i = 0; i < 8; i++) printf("0x%02X ", frame.data[i]);
        printf("]\n");
        fflush(stdout);

        int nbytes = write(can_sock, &frame, sizeof(struct can_frame));
        if (nbytes < 0) perror("CAN send");

        ros::spinOnce();
        loop_rate.sleep();
    }
    close(can_sock);
    return 0;
}
