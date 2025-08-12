#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16, Bool
import can

class CanTrNode:
    def __init__(self):
        rospy.init_node('can_tr_node', anonymous=True)

        self.current_rpm = 0
        self.current_steering = 90  # 초기 센터값
        self.command = 0x82         # 기본 전진 명령 (0x82: 전진, 0x02: 후진, 0x00: 정지)

        # ROS subscribers
        rospy.Subscriber("/can/rpm", Int16, self.rpm_callback)
        rospy.Subscriber("/can/steer", Int16, self.steering_callback)
        rospy.Subscriber("/can/dir", Bool, self.command_callback)
        rospy.Subscriber("/can/stop", Bool, self.stop_callback)

        # CAN 인터페이스 초기화 (socketcan 사용, 인터페이스 이름 "can0")
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        except OSError:
            rospy.logerr("Cannot find CAN interface 'can0'. Make sure it is up.")
            exit(1)

        self.rate = rospy.Rate(10)  # 10 Hz 송출

    def rpm_callback(self, msg):
        rpm = msg.data
        if rpm < 0:
            rpm = 0
        if rpm > 16000:
            rpm = 16000
        self.current_rpm = (rpm // 100) * 100  # 100 단위 제한

    def steering_callback(self, msg):
        steer = msg.data
        if steer < 69:
            steer = 69
        if steer > 111:
            steer = 111
        self.current_steering = steer

    def command_callback(self, msg):
        # True = 전진, False = 후진
        self.command = 0x82 if msg.data else 0x02

    def stop_callback(self, msg):
        if msg.data:
            self.command = 0x00  # 정지 명령

    def send_can_frame(self):
        # CAN 데이터 패킷 구성
        rpm_val = self.current_rpm
        data = [0] * 8
        data[0] = self.command
        data[1] = (rpm_val >> 8) & 0xFF
        data[2] = rpm_val & 0xFF
        data[3] = 0xAA
        data[4] = self.current_steering
        data[5] = 0x00
        data[6] = 0x00
        data[7] = 0x00

        msg = can.Message(arbitration_id=0x01, data=data, is_extended_id=False)

        try:
            self.bus.send(msg)
            rospy.loginfo(f"Sent CAN frame: CMD=0x{data[0]:02X} RPM={rpm_val} STEER={self.current_steering}")
        except can.CanError:
            rospy.logerr("CAN write error")

    def run(self):
        while not rospy.is_shutdown():
            self.send_can_frame()
            self.rate.sleep()


if __name__ == '__main__':
    node = CanTrNode()
    node.run()
