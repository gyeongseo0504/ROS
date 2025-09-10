// 통합된 henes_car_control_traffic_integrated.cpp
#define DEBUG 0
#define DEBUG_ROS_INFO 1

#include "ros/ros.h"
#include <memory>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/TransformStamped.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <ctime>

// -------------------- 유틸/매크로 --------------------
#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
#define LOOP_RATE 50

template<typename T>
static inline T clampv(T v, T lo, T hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// -------------------- CAN 정의 --------------------
#define CAN_INTERFACE "can0"
#define CAN_COMBINED_ID 0x01
#define Right_MAX -24
#define Left_MAX 25
#define CAN_SEND_INTERVAL_MS 100

// -------------------- 전역상태 --------------------
static int uart_fd = -1;
int steering_angle = 0;
int steering_angle_cmd = 0;
float motor_speed = 0.0f;
float motor_speed_cmd = 0.0f;
int steering_angle_old = 0;
float motor_speed_old = 0.0f;
double m_acceleration = 0.5;
double m_deceleration = 0.5;
double s_acceleration = 1.0;
double s_deceleration = 1.0;
double yaw = 0.0, yaw_old = 0.0;

static tf::TransformBroadcaster* tf_broadcaster = nullptr;

// -------------------- 신호등 제어 상태 --------------------
enum VehicleState {
    UNDETECTED,
    STOPPED_RED,
    STOPPED_YELLOW,
    STOPPED_GREEN_WAIT,
    MOVING_AFTER_SIGNAL
};

static VehicleState vehicle_state = UNDETECTED;
static std::string current_light_status = "unknown";
static std::string detection_source = "";
static double undetected_start_time = 0.0;
static double green_wait_start_time = 0.0;
static const double undetected_duration = 3.0;
static const double green_wait_duration = 2.0;
static int target_rpm = 500;
static int steering_angle_param = 93;
static bool traffic_control_override = false; // 신호등 제어 우선순위

struct BaseSensorData {
    int encoder = 0;
    int encoder_old = 0;
} myBaseSensorData;

union {
    int data;
    unsigned char bytedata[4];
} m_car_encoder_int;

union {
    short data;
    unsigned char bytedata[2];
} m_robot_angle_int16;

union {
    float data;
    unsigned char bytedata[4];
} m_current_robot_speed_float;

struct OdomCalculateData {
    float distance_ratio = 348.0f;
    float encode_sampling_time = 0.05f;
    float position_x = 0.0f;
    float position_y = 0.0f;
    float oriention = 0.0f;
} myOdomCalculateData;

// -------------------- 조향 HEX 테이블 --------------------
static const unsigned char STEER_HEX_LIST[47] = {
    0x00, 0x06, 0x0B, 0x11, 0x16, 0x1C, 0x21, 0x27, 0x2C, 0x32, 0x38,
    0x3D, 0x43, 0x48, 0x4E, 0x53, 0x59, 0x5E, 0x64, 0x6A, 0x6F, 0x75,
    0x7A, 0x80, 0x85, 0x8B, 0x90, 0x96, 0x9C, 0xA1, 0xA7, 0xAC, 0xB2,
    0xB7, 0xBD, 0xC2, 0xC8, 0xCE, 0xD3, 0xD9, 0xDE, 0xE4, 0xE9, 0xEF,
    0xF4, 0xFA, 0xFF
};

static inline unsigned char angle_to_hex(int angle) {
    angle = clampv(angle, 68, 114);
    return STEER_HEX_LIST[angle - 68];
}

// -------------------- 시리얼 --------------------
static void write_serial(const unsigned char *buf, int len) {
    if (uart_fd < 0) return;
    int wrote = 0;
    while (wrote < len) {
        int n = write(uart_fd, buf + wrote, len - wrote);
        if (n <= 0) break;
        wrote += n;
    }
}

static int init_serial_port(void) {
    uart_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd == -1) {
        ROS_WARN("Unable to open serial port /dev/ttyUSB0 (serial optional)");
        return -1;
    }

    struct termios opt{};
    tcgetattr(uart_fd, &opt);
    cfsetispeed(&opt, B115200);
    cfsetospeed(&opt, B115200);
    opt.c_cflag |= (CLOCAL | CREAD);
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opt.c_oflag &= ~OPOST;
    opt.c_cc[VMIN] = 0;
    opt.c_cc[VTIME] = 0;
    tcsetattr(uart_fd, TCSANOW, &opt);
    return uart_fd;
}

static void* readserial_thread(void* /*unused*/) {
    unsigned char buf[8] = {0};
    unsigned char b = 0;
    while (true) {
        if (uart_fd < 0) { usleep(1000); continue; }
        int n = read(uart_fd, &b, 1);
        if (n == 1) {
            for (int i=0;i<7;++i) buf[i] = buf[i+1];
            buf[7] = b;
            if (buf[0]=='#' && buf[1]=='E' && buf[7]=='*') {
                m_car_encoder_int.bytedata[0] = buf[2];
                m_car_encoder_int.bytedata[1] = buf[3];
                m_car_encoder_int.bytedata[2] = buf[4];
                m_car_encoder_int.bytedata[3] = buf[5];
                ROS_DEBUG("ENC RX: %d", m_car_encoder_int.data);
            }
        } else {
            usleep(1000);
        }
    }
    return nullptr;
}

static void send_serial_data() {
    unsigned char pkt[9]{};
    pkt[0] = '#'; pkt[1] = 'C';
    pkt[2] = m_robot_angle_int16.bytedata[0];
    pkt[3] = m_robot_angle_int16.bytedata[1];
    pkt[4] = m_current_robot_speed_float.bytedata[0];
    pkt[5] = m_current_robot_speed_float.bytedata[1];
    pkt[6] = m_current_robot_speed_float.bytedata[2];
    pkt[7] = m_current_robot_speed_float.bytedata[3];
    pkt[8] = '*';
    write_serial(pkt, 9);
}

// -------------------- CAN Controller --------------------
class CANController {
    int can_sock_{-1};
    struct sockaddr_can addr_{};
    struct ifreq ifr_{};
    ros::Time last_send_{};

public:
    ~CANController() { if (can_sock_>=0) close(can_sock_); }
    
    bool initialize() {
        can_sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_sock_ < 0) { ROS_ERROR("CAN socket create failed"); return false; }

        std::strcpy(ifr_.ifr_name, CAN_INTERFACE);
        if (ioctl(can_sock_, SIOCGIFINDEX, &ifr_) < 0) { ROS_ERROR("SIOCGIFINDEX failed"); return false; }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;
        if (bind(can_sock_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) { ROS_ERROR("CAN bind failed"); return false; }

        ROS_INFO("CAN ready on %s, unified ID 0x%02X", CAN_INTERFACE, CAN_COMBINED_ID);
        return true;
    }

    void sendCombined(int steer_deg, float speed_rpm) {
        if (last_send_.isZero()) last_send_ = ros::Time::now();
        ros::Time now = ros::Time::now();
        double ms = (now - last_send_).toSec() * 1000.0;

        if (ms < CAN_SEND_INTERVAL_MS &&
            steer_deg == steering_angle_old &&
            std::fabs(speed_rpm - motor_speed_old) < 0.1f) {
            return;
        }

        struct can_frame f{};
        f.can_id = CAN_COMBINED_ID;
        f.can_dlc = 8;

        // B0: 방향
        if (speed_rpm > 0) f.data[0] = 0x82;
        else if (speed_rpm < 0) f.data[0] = 0x02;
        else f.data[0] = 0x00;

        // B1-B2: RPM(LE)
        int rpm = clampv((int)std::llround(std::fabs(speed_rpm)), 0, 16000);
        f.data[1] = (unsigned char)(rpm & 0xFF);
        f.data[2] = (unsigned char)((rpm >> 8) & 0xFF);

        // B3: 0xAA
        f.data[3] = 0xAA;

        // B4: 조향 각도
        int lim = clampv(steer_deg, Right_MAX, Left_MAX);
        int servo_angle = (int)std::llround(68.0 + ((lim - Right_MAX) * 44.0) / (Left_MAX - Right_MAX));
        f.data[4] = angle_to_hex(servo_angle);

        // B5-B7: padding
        f.data[5] = f.data[6] = f.data[7] = 0x00;

        int wr = write(can_sock_, &f, sizeof(f));
        if (wr != (int)sizeof(f)) {
            ROS_WARN("CAN send failed");
        } else {
            ROS_INFO_THROTTLE(0.5,
                "[CAN 0x%02X] dir=0x%02X rpm=%d steer_hex=0x%02X (cmd=%d) [%s]",
                CAN_COMBINED_ID, f.data[0], rpm, f.data[4], lim, 
                traffic_control_override ? "TRAFFIC" : "WAYPOINT");
        }

        last_send_ = now;
    }
};

// -------------------- 신호등 제어 로직 --------------------
static void handle_signal_detection(const std::string& status) {
    if (status == "traffic_red") {
        vehicle_state = STOPPED_RED;
        motor_speed_cmd = 0;
        traffic_control_override = true;
        ROS_INFO("🔴 빨간불 감지 - 신호등 제어 활성화 (정지)");
    } else if (status == "traffic_yellow") {
        vehicle_state = STOPPED_YELLOW;
        motor_speed_cmd = 0;
        traffic_control_override = true;
        ROS_INFO("🟡 노간불 감지 - 신호등 제어 활성화 (정지)");
    } else if (status == "traffic_green") {
        vehicle_state = STOPPED_GREEN_WAIT;
        motor_speed_cmd = 0;
        traffic_control_override = true;
        green_wait_start_time = ros::Time::now().toSec();
        ROS_INFO("🟢 초록불 감지 - 2초 대기 시작");
    }
}

static void enter_undetected_state() {
    VehicleState previous_state = vehicle_state;
    vehicle_state = UNDETECTED;
    current_light_status = "unknown";
    green_wait_start_time = 0;
    undetected_start_time = 0;
    traffic_control_override = false; // 웨이포인트 제어 복원
    ROS_INFO("🔄 미인식 단계 진입 완료 (%d → UNDETECTED) - 웨이포인트 제어 복원", previous_state);
}

static void start_moving() {
    green_wait_start_time = 0;
    traffic_control_override = false; // 웨이포인트 제어 복원
    ROS_INFO("🚀 출발 - 웨이포인트 제어 복원");
}

// -------------------- TF 브로드캐스트 함수 --------------------
static void broadcast_tf(const ros::Time& timestamp) {
    if (!tf_broadcaster) return;
    try {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = timestamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = myOdomCalculateData.position_x;
        odom_trans.transform.translation.y = myOdomCalculateData.position_y;
        odom_trans.transform.translation.z = 0.0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(myOdomCalculateData.oriention);
        odom_trans.transform.rotation = odom_quat;

        tf_broadcaster->sendTransform(odom_trans);
        ROS_DEBUG("TF broadcast: odom->base_link [%.3f, %.3f, %.3f°]",
                  myOdomCalculateData.position_x,
                  myOdomCalculateData.position_y,
                  RAD2DEG(myOdomCalculateData.oriention));
    } catch (const std::exception& e) {
        ROS_WARN("TF broadcast failed: %s", e.what());
    }
}

// -------------------- 콜백 & 보조 --------------------
static void imu_heading_angle_cb(const std_msgs::Float32& msg) {
    yaw = msg.data;
}

static void twist_cb(const geometry_msgs::Twist& msg) {
    if (!traffic_control_override) {
        steering_angle_cmd = clampv((int)msg.angular.z, Right_MAX, Left_MAX);
        motor_speed_cmd = clampv((float)msg.linear.x, -16000.f, 16000.f);
        ROS_DEBUG("TWIST CMD: steer=%d speed=%.1f", steering_angle_cmd, motor_speed_cmd);
    }
}

static void steer_cb(const std_msgs::Int16& a) {
    if (!traffic_control_override) {
        steering_angle_cmd = clampv((int)a.data, Right_MAX, Left_MAX);
        ROS_DEBUG("STEER CMD: %d", steering_angle_cmd);
    }
}

static void speed_cb(const std_msgs::Float32& s) {
    if (!traffic_control_override) {
        motor_speed_cmd = clampv((float)s.data, -16000.f, 16000.f);
        ROS_DEBUG("SPEED CMD: %.1f rpm", motor_speed_cmd);
    }
}

// ★ 웨이포인트에서 오는 조향 명령 콜백
static void waypoint_steer_cb(const std_msgs::Int16& msg) {
    if (!traffic_control_override) {
        steering_angle_cmd = clampv((int)msg.data, Right_MAX, Left_MAX);
        ROS_DEBUG("Waypoint STEER CMD: %d", steering_angle_cmd);
    }
}

// ★ 웨이포인트에서 오는 속도 명령 콜백
static void waypoint_speed_cb(const std_msgs::Float32& msg) {
    if (!traffic_control_override) {
        motor_speed_cmd = clampv((float)msg.data, -16000.f, 16000.f);
        ROS_DEBUG("Waypoint SPEED CMD: %.1f rpm", motor_speed_cmd);
    }
}

// ★ 웨이포인트에서 오는 목표 각도 콜백
static void waypoint_target_angle_cb(const std_msgs::Float32& msg) {
    if (!traffic_control_override) {
        // 목표 각도를 조향각으로 변환 (각도 제어 방식)
        steering_angle_cmd = clampv((int)msg.data, Right_MAX, Left_MAX);
        ROS_DEBUG("Waypoint TARGET ANGLE CMD: %.1f -> %d", msg.data, steering_angle_cmd);
    }
}

// ★ 신호등 감지 콜백
static void traffic_callback(const std_msgs::String& msg) {
    try {
        std::string data = msg.data;
        size_t pos = 0;
        std::vector<std::string> parts;
        std::string token;
        
        while ((pos = data.find('|')) != std::string::npos) {
            token = data.substr(0, pos);
            parts.push_back(token);
            data.erase(0, pos + 1);
        }
        parts.push_back(data); // 마지막 부분
        
        if (parts.size() >= 4) {
            std::string detection_status = parts[0];
            std::string light_status = parts[1];
            float confidence = std::stof(parts[2]);
            std::string source = parts[3];
            
            bool detected = (detection_status == "detected");
            
            double current_time = ros::Time::now().toSec();
            
            // 신호등 감지/미감지에 따른 상태 전환
            if (!detected) {
                if (undetected_start_time == 0) {
                    undetected_start_time = current_time;
                } else if ((current_time - undetected_start_time) >= undetected_duration) {
                    if (vehicle_state != UNDETECTED) {
                        ROS_INFO("🔄 3초 연속 미인식 - 미인식 단계 진입");
                        enter_undetected_state();
                    }
                }
            } else {
                undetected_start_time = 0;
                current_light_status = light_status;
                detection_source = source;
                
                if (vehicle_state == UNDETECTED) {
                    handle_signal_detection(light_status);
                } else if (vehicle_state == STOPPED_GREEN_WAIT) {
                    if (light_status == "traffic_red" || light_status == "traffic_yellow") {
                        if (light_status == "traffic_red") {
                            vehicle_state = STOPPED_RED;
                            ROS_INFO("🔴 초록불 대기 중 빨간불 감지 - 빨간불 정지 상태로 전환");
                        } else {
                            vehicle_state = STOPPED_YELLOW;
                            ROS_INFO("🟡 초록불 대기 중 노간불 감지 - 노간불 정지 상태로 전환");
                        }
                        green_wait_start_time = 0;
                    }
                } else if (vehicle_state == STOPPED_RED || vehicle_state == STOPPED_YELLOW) {
                    if (light_status == "traffic_green") {
                        start_moving();
                        vehicle_state = MOVING_AFTER_SIGNAL;
                        ROS_INFO("🟢 초록불 감지 - 바로 출발!");
                    }
                }
            }
            
            // 초록불 대기 시간 확인
            if (vehicle_state == STOPPED_GREEN_WAIT && green_wait_start_time > 0) {
                double elapsed = current_time - green_wait_start_time;
                if (elapsed >= green_wait_duration) {
                    start_moving();
                    vehicle_state = MOVING_AFTER_SIGNAL;
                    ROS_INFO("🚀 2초 대기 완료 - 신호 통과 후 주행 시작!");
                }
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN("신호등 메시지 파싱 실패: %s", e.what());
    }
}

static void robot_steer_profile_control() {
    if (steering_angle > steering_angle_cmd) {
        steering_angle -= (int)(s_deceleration / LOOP_RATE);
        if (steering_angle < steering_angle_cmd) steering_angle = steering_angle_cmd;
    } else if (steering_angle < steering_angle_cmd) {
        steering_angle += (int)(s_acceleration / LOOP_RATE);
        if (steering_angle > steering_angle_cmd) steering_angle = steering_angle_cmd;
    } else {
        steering_angle = steering_angle_cmd;
    }
}

static void robot_speed_profile_control() {
    if (motor_speed > motor_speed_cmd) {
        motor_speed -= (m_deceleration / LOOP_RATE);
        if (motor_speed < motor_speed_cmd) motor_speed = motor_speed_cmd;
    } else if (motor_speed < motor_speed_cmd) {
        motor_speed += (m_acceleration / LOOP_RATE);
        if (motor_speed > motor_speed_cmd) motor_speed = motor_speed_cmd;
    } else {
        motor_speed = motor_speed_cmd;
    }
}

static void odometry_cal() {
    int denc = myBaseSensorData.encoder - myBaseSensorData.encoder_old;
    double dl = (double)denc;
    double dyaw = yaw - yaw_old;
    double bx, by;
    
    if (std::fabs(dyaw) > 1e-7) {
        double R = dl / dyaw;
        by = ((dyaw < 0) ? -1.0 : 1.0) * R * (1 - std::cos(dyaw));
        bx = R * std::sin(dyaw);
    } else {
        bx = dl; by = 0.0;
    }

    bx /= myOdomCalculateData.distance_ratio;
    by /= myOdomCalculateData.distance_ratio;
    double ox = bx * std::cos(yaw_old) - by * std::sin(yaw_old);
    double oy = bx * std::sin(yaw_old) + by * std::cos(yaw_old);
    
    myOdomCalculateData.position_x += (float)ox;
    myOdomCalculateData.position_y += (float)oy;
    myOdomCalculateData.oriention = (float)yaw;
    yaw_old = yaw;
}

// -------------------- main --------------------
// 기존 코드는 그대로 유지하고, main 함수 부분만 수정

int main(int argc, char **argv) {
    ros::init(argc, argv, "henes_car_control_traffic_integrated");
    ros::NodeHandle nh;
    
    tf_broadcaster = new tf::TransformBroadcaster();
    ROS_INFO("TF broadcaster initialized");
    
    setvbuf(stdout, NULL, _IONBF, 0);
    
    // 파라미터 로드
    ros::param::get("~m_acceleration", m_acceleration);
    ros::param::get("~m_deceleration", m_deceleration);
    ros::param::get("~s_acceleration", s_acceleration);
    ros::param::get("~s_deceleration", s_deceleration);
    ros::param::get("~target_rpm", target_rpm);
    ros::param::get("~steering_angle", steering_angle_param);
    
    // 시리얼 초기화 (옵션)
    uart_fd = init_serial_port();
    if (uart_fd >= 0) {
        pthread_t th;
        if (pthread_create(&th, nullptr, readserial_thread, nullptr) != 0) {
            ROS_WARN("readserial_thread create failed");
        }
    }
    
    // CAN 초기화
    std::unique_ptr<CANController> can(new CANController());
    if (!can->initialize()) {
        ROS_ERROR("CAN init failed");
        return -1;
    }
    
    // 초기 설정
    motor_speed_cmd = target_rpm;
    steering_angle_cmd = 0;
    
    // 구독자들
    ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1, &twist_cb);
    ros::Subscriber sub_steer = nh.subscribe("/Car_Control_Cmd/steerAngle_Int16", 1, &steer_cb);
    ros::Subscriber sub_speed = nh.subscribe("/Car_Control_Cmd/speed_Float32", 1, &speed_cb);
    ros::Subscriber sub_imu = nh.subscribe("/imu/heading_angle_radian", 10, &imu_heading_angle_cb);
    
    // 웨이포인트 제어 구독자들 (핵심 수정!)
    ros::Subscriber sub_wp_steer = nh.subscribe("/Car_Control_Cmd/xte_steerAngle_Int16", 1, &waypoint_steer_cb);
    ros::Subscriber sub_wp_speed = nh.subscribe("/Car_Control_Cmd/speed_Float32", 1, &waypoint_speed_cb);
    ros::Subscriber sub_wp_target_angle = nh.subscribe("/Car_Control_Cmd/Target_Angle", 1, &waypoint_target_angle_cb);
    
    // 신호등 감지 구독자
    ros::Subscriber sub_traffic = nh.subscribe("/traffic_light_detection", 10, &traffic_callback);
    
    // 발행자들
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom/car", 20);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("/vehicle_status", 10);
    
    // 오도메트리 설정
    std::string odom_frame_id = "odom";
    std::string odom_child_frame_id = "base_link";
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion q;
    
    ros::Rate r(LOOP_RATE);
    ros::Duration(0.5).sleep();
    
    ROS_INFO("========================================");
    ROS_INFO("✅ Integrated Henes Car Control Started");
    ROS_INFO("🚦 Traffic Light: Override Mode");
    ROS_INFO("🗺️ Waypoint: Normal Mode");
    ROS_INFO("CAN ID: 0x001 (Combined Control)");
    ROS_INFO("========================================");
    
    while (ros::ok()) {
        robot_speed_profile_control();
        robot_steer_profile_control();
        
        // 상태 로그
        ROS_INFO_THROTTLE(1.0,
            "CMD/ACT steer: %d -> %d | speed: %.1f -> %.1f rpm | State: %d | Control: %s",
            steering_angle_cmd, steering_angle, motor_speed_cmd, motor_speed,
            vehicle_state, traffic_control_override ? "TRAFFIC" : "WAYPOINT");
        
        // CAN ID 0x01 프레임 송신
        can->sendCombined(steering_angle, motor_speed);
        
        // 시리얼 송신 (옵션)
        if (steering_angle != steering_angle_old || std::fabs(motor_speed - motor_speed_old) > 1e-3) {
            m_robot_angle_int16.data = (short)steering_angle;
            m_current_robot_speed_float.data = motor_speed;
            send_serial_data();
        }
        
        steering_angle_old = steering_angle;
        motor_speed_old = motor_speed;
        
        // 엔코더 -> 오도메트리
        myBaseSensorData.encoder_old = myBaseSensorData.encoder;
        myBaseSensorData.encoder = m_car_encoder_int.data;
        odometry_cal();
        
        // 현재 시간
        ros::Time current_time = ros::Time::now();
        
        // TF 브로드캐스트
        broadcast_tf(current_time);
        
        // 오도메트리 발행
        q = tf::createQuaternionMsgFromYaw(myOdomCalculateData.oriention);
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = odom_child_frame_id;
        odom.pose.pose.position.x = myOdomCalculateData.position_x;
        odom.pose.pose.position.y = myOdomCalculateData.position_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = q;
        odom.twist.twist.linear.x = motor_speed / myOdomCalculateData.distance_ratio;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = (yaw - yaw_old) * LOOP_RATE;
        odom_pub.publish(odom);
        
        // 차량 상태 발행
        std_msgs::String status_msg;
        status_msg.data = std::to_string(vehicle_state) + "|" + current_light_status + "|" + 
                         detection_source + "|" + (traffic_control_override ? "TRAFFIC" : "WAYPOINT");
        status_pub.publish(status_msg);
        
        ros::spinOnce();
        r.sleep();
    }
    
    // 정리
    if (tf_broadcaster) {
        delete tf_broadcaster;
        tf_broadcaster = nullptr;
    }
    
    motor_speed_cmd = 0;
    steering_angle_cmd = 0;
    ROS_INFO("프로그램 종료 - 차량 정지");
    return 0;
}
