#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Int16, Float32
import can
import math

class CanImuGpsNode:
    def __init__(self):
        rospy.init_node('can_imu_gps_node', anonymous=True)

        # IMU-GPS 보정 변수
        self.imu1_yaw = 0.0
        self.imu2_yaw = 0.0
        self.gps_heading = 0.0
        self.imu_offset_deg = 0.0
        self.corrected_imu_yaw = 0.0

        self.choose_imu_no = rospy.get_param('~choose_imu_no', 1)
        self.auto_imu_offset_flag = rospy.get_param('~auto_imu_offset_flag', True)
        self.enable_imu_correction = True

        # CAN 송신 제어 변수
        self.current_rpm = 0
        self.current_steering = 90
        self.command = 0x82  # 기본 전진 명령

        # 받은 302 피드백 값 저장 변수
        self.feedback_302_data = None

        # 토픽명
        self.imu1_yaw_topic = rospy.get_param('~imu1_yaw_radian_topic', '/handsfree/imu/yaw_radian')
        self.imu2_yaw_topic = rospy.get_param('~imu2_yaw_radian_topic', '/robor/imu/yaw_radian')
        self.gps_heading_topic = rospy.get_param('~gps_heading_angle', '/gps/heading_angle')
        self.imu_correction_enable_topic = rospy.get_param('~imu_correction_enable_topic', '/flag/imu_auto_correction')
        self.imu_offset_angle_topic = rospy.get_param('~imu_heading_angle_offset_degree_topic', '/imu/heading_angle_offset_degree')
        self.corrected_imu_yaw_topic = rospy.get_param('~imu_heading_angle_radian_topic', '/imu/heading_angle_radian')

        self.rpm_topic = '/Car_Control_Cmd/speed_Float32'         
        self.steering_topic = '/Car_Control_Cmd/SteerAngle_Int16'  
        self.direction_topic = '/Car_Control_Cmd/drive_direction' 
        self.stop_topic = '/Car_Control_Cmd/stop'                 

        # ROS Subscribers for IMU-GPS yaw correction
        rospy.Subscriber(self.imu1_yaw_topic, Float32, self.imu1_yaw_callback)
        rospy.Subscriber(self.imu2_yaw_topic, Float32, self.imu2_yaw_callback)
        rospy.Subscriber(self.gps_heading_topic, Float32, self.gps_heading_callback)
        rospy.Subscriber(self.imu_correction_enable_topic, Bool, self.enable_correction_callback)

        # ROS Subscribers for CAN command
        rospy.Subscriber(self.rpm_topic, Float32, self.rpm_callback)
        rospy.Subscriber(self.steering_topic, Int16, self.steering_callback)
        rospy.Subscriber(self.direction_topic, Bool, self.direction_callback)
        rospy.Subscriber(self.stop_topic, Bool, self.stop_callback)

        # Publishers for IMU offset and corrected yaw
        self.pub_imu_offset_angle = rospy.Publisher(self.imu_offset_angle_topic, Float32, queue_size=1)
        self.pub_corrected_imu_yaw = rospy.Publisher(self.corrected_imu_yaw_topic, Float32, queue_size=1)

        # 302 피드백 데이터 퍼블리셔
        self.pub_feedback_302 = rospy.Publisher('/can/feedback_302', Float32, queue_size=1)

        # CAN interface setup
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        except OSError:
            rospy.logerr("Cannot find CAN interface 'can0'. Make sure 'can0' is up.")
            exit(1)

        # Rates
        self.rate_hz = 50
        self.can_rate_hz = 10

        self.last_can_send_time = rospy.Time.now()

    # IMU-GPS Callbacks
    def imu1_yaw_callback(self, msg):
        self.imu1_yaw = msg.data

    def imu2_yaw_callback(self, msg):
        self.imu2_yaw = msg.data

    def gps_heading_callback(self, msg):
        self.gps_heading = msg.data

    def enable_correction_callback(self, msg):
        self.enable_imu_correction = msg.data

    # CAN command Callbacks
    def rpm_callback(self, msg):
        rpm = int(msg.data)
        rpm = max(0, min(16000, rpm))
        self.current_rpm = (rpm // 100) * 100

    def steering_callback(self, msg):
        steer = msg.data
        steer = max(69, min(111, steer))
        self.current_steering = steer

    def direction_callback(self, msg):
        self.command = 0x82 if msg.data else 0x02

    def stop_callback(self, msg):
        if msg.data:
            self.command = 0x00

    def imu_gps_correction(self):
        imu_yaw = self.imu1_yaw if self.choose_imu_no == 1 else self.imu2_yaw

        if not self.auto_imu_offset_flag:
            imu_offset_rad = rospy.get_param('~imu_yaw_offset_degree', 0.0) * (math.pi / 180.0)
            self.corrected_imu_yaw = imu_yaw + imu_offset_rad
        else:
            if self.enable_imu_correction:
                imu_offset_rad = self.gps_heading - imu_yaw
                self.imu_offset_deg = imu_offset_rad * 180.0 / math.pi
                self.corrected_imu_yaw = imu_yaw + imu_offset_rad
                self.pub_imu_offset_angle.publish(self.imu_offset_deg)
            else:
                self.corrected_imu_yaw = imu_yaw + (self.imu_offset_deg * math.pi / 180.0)

        # Normalize to [-pi, pi]
        if self.corrected_imu_yaw > math.pi:
            self.corrected_imu_yaw -= 2 * math.pi
        elif self.corrected_imu_yaw < -math.pi:
            self.corrected_imu_yaw += 2 * math.pi

        self.pub_corrected_imu_yaw.publish(self.corrected_imu_yaw)

    def send_can_message(self):
        data = [0] * 8
        data[0] = self.command
        data[1] = (self.current_rpm >> 8) & 0xFF
        data[2] = self.current_rpm & 0xFF
        data[3] = 0xAA
        data[4] = self.current_steering
        data[5] = 0x00
        data[6] = 0x00
        data[7] = 0x00

        msg = can.Message(arbitration_id=0x01, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
            rospy.loginfo(f"Sent CAN frame: CMD=0x{data[0]:02X} RPM={self.current_rpm} STEER={self.current_steering}")
        except can.CanError:
            rospy.logerr("Failed to send CAN message")

    def receive_can_feedback(self):
        """
        CAN 버스에서 0x302 ID의 피드백 메시지를 수신
        """
        msg = self.bus.recv(timeout=0.001)  # non-blocking
        if msg is not None and msg.arbitration_id == 0x302:
            # 예시: data[0]~data[1]를 하나의 값으로 처리
            feedback_value = (msg.data[0] << 8) | msg.data[1]
            self.feedback_302_data = feedback_value
            rospy.loginfo(f"Received 0x302 feedback: {feedback_value}")
            # ROS 토픽으로 퍼블리시
            self.pub_feedback_302.publish(float(feedback_value))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.imu_gps_correction()

            # 주기적으로 CAN 송신
            if (now - self.last_can_send_time).to_sec() >= 0.1:
                self.send_can_message()
                self.last_can_send_time = now

            # CAN 피드백 수신
            self.receive_can_feedback()

            rate.sleep()


if __name__ == '__main__':
    node = CanImuGpsNode()
    node.run()

