#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import can
import struct
import threading
import time

# === 조향 각도 매핑 테이블 (69~111도) ===
STEER_HEX_LIST = [
    0x00, 0x06, 0x0C, 0x12, 0x18, 0x1E, 0x24, 0x2A, 0x30, 0x36, 0x3C,
    0x42, 0x48, 0x4E, 0x54, 0x5A, 0x60, 0x66, 0x6C, 0x72, 0x78, 0x7E,
    0x84, 0x8A, 0x90, 0x96, 0x9C, 0xA2, 0xA8, 0xAE, 0xB4, 0xBA, 0xC0,
    0xC6, 0xCC, 0xD2, 0xD8, 0xDE, 0xE4, 0xEA, 0xF0, 0xF6, 0xFC
]
STEER_MIN_DEG = 69
STEER_MAX_DEG = 111

def map_steering_angle(degree):
    """조향 각도(실수)를 16진수 테이블 값으로 변환 (69~111: index 0~42, 클램핑)"""
    degree_int = int(round(degree))
    if degree_int <= STEER_MIN_DEG:
        return STEER_HEX_LIST[0]
    elif degree_int >= STEER_MAX_DEG:
        return STEER_HEX_LIST[-1]
    else:
        idx = degree_int - STEER_MIN_DEG
        return STEER_HEX_LIST[idx]

class CANController:
    def __init__(self):
        rospy.init_node('can_controller', anonymous=True)

        # CAN 인터페이스 설정
        self.can_interface = rospy.get_param('~can_interface', 'can0')
        self.can_bitrate = rospy.get_param('~can_bitrate', 500000)
        self.send_period_ms = rospy.get_param('~send_period_ms', 100)   # 기본 100ms

        # 제어 변수 초기화
        self.target_rpm = 0.0     # 목표 RPM (0~16000)
        self.actual_rpm = 0.0     # 실제 RPM (피드백)
        self.current_duty = 0     # 현재 듀티 (피드백)
        self.steering = 90.0      # 90도가 정중앙
        self.is_running = True

        # CAN 버스 초기화
        try:
            self.bus = can.interface.Bus(channel=self.can_interface,
                                         bustype='socketcan',
                                         bitrate=self.can_bitrate)
            rospy.loginfo("CAN bus initialized on %s (bitrate: %d)", self.can_interface, self.can_bitrate)
        except Exception as e:
            rospy.logerr("Failed to initialize CAN bus: %s", str(e))
            rospy.signal_shutdown("CAN initialization failed")
            self.bus = None
            return

        # Publisher/Subscriber 설정
        self.control_sub = rospy.Subscriber('/can_control_cmd',
                                            Float32MultiArray,
                                            self.control_callback)

        self.status_pub = rospy.Publisher('/can_status',
                                          Float32MultiArray,
                                          queue_size=10)

        # 디버그 정보 Publisher
        self.debug_pub = rospy.Publisher('/can_debug',
                                        Float32MultiArray,
                                        queue_size=10)

        # CAN 송신/수신 스레드 시작
        self.send_thread = threading.Thread(target=self.send_can_periodic)
        self.send_thread.daemon = True
        self.send_thread.start()

        self.receive_thread = threading.Thread(target=self.receive_can)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        rospy.loginfo("CAN Controller initialized successfully (PID Mode)")

    def control_callback(self, msg):
        """키보드 입력으로부터 제어 명령 수신"""
        if len(msg.data) >= 2:
            self.target_rpm = msg.data[0]
            self.steering = msg.data[1]
            
            # RPM 제한 (-16000 ~ +16000)
            if self.target_rpm > 16000:
                self.target_rpm = 16000
            elif self.target_rpm < -16000:
                self.target_rpm = -16000
                
            rospy.logdebug("Received control: Target RPM=%.1f, Steering=%.1f",
                           self.target_rpm, self.steering)

    def send_can_periodic(self):
        """주기적으로 CAN 메시지 전송 (조향 매핑 포함)"""
        period = self.send_period_ms / 1000.0
        while not rospy.is_shutdown() and self.is_running:
            try:
                cmd_id = 0x82  # 항상 0x82

                # 2,3바이트: 목표 RPM (signed, little-endian)
                rpm_int = int(self.target_rpm)
                rpm_bytes = struct.pack('<h', rpm_int)

                # 4번째 바이트: 항상 0xAA
                steering_enable = 0xAA

                # 5번째 바이트: 조향 각도 매핑값
                steering_angle = map_steering_angle(self.steering)

                # 6~8바이트: 0
                data = [
                    cmd_id,
                    rpm_bytes[0], rpm_bytes[1],
                    steering_enable,
                    steering_angle,
                    0, 0, 0
                ]

                msg = can.Message(arbitration_id=0x001, data=data, is_extended_id=False)

                try:
                    self.bus.send(msg, timeout=0.1)
                    rospy.logdebug("Sent CAN: ID=0x%03X, CMD=0x%02X, RPM=%d, Enable=0x%02X, Steer=0x%02X",
                                   msg.arbitration_id, cmd_id,
                                   rpm_int, steering_enable, steering_angle)
                except can.CanError as ce:
                    rospy.logwarn("CAN send failed (retrying): %s", str(ce))
                except Exception as se:
                    rospy.logerr("Unexpected error on CAN send: %s", str(se))
            except Exception as e:
                rospy.logerr("Critical error in send_can_periodic: %s", str(e))

            time.sleep(period)

    def receive_can(self):
        """CAN 메시지 수신 (ID 0x300)"""
        while not rospy.is_shutdown() and self.is_running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is not None and msg.arbitration_id == 0x300:
                    # TC275에서 보낸 피드백 데이터 파싱
                    # data[0]: cmd_id
                    # data[1]: current_duty
                    # data[2:3]: actual_rpm (Little-endian)
                    # data[4:5]: target_rpm (Little-endian)
                    # data[6]: steer_angle
                    
                    cmd_id = msg.data[0]
                    self.current_duty = msg.data[1]
                    
                    # 실제 RPM (signed 16-bit)
                    rpm_bytes = msg.data[2] | (msg.data[3] << 8)
                    if rpm_bytes > 32767:
                        self.actual_rpm = rpm_bytes - 65536
                    else:
                        self.actual_rpm = rpm_bytes
                    
                    # 목표 RPM (signed 16-bit)
                    target_bytes = msg.data[4] | (msg.data[5] << 8)
                    if target_bytes > 32767:
                        received_target_rpm = target_bytes - 65536
                    else:
                        received_target_rpm = target_bytes
                    
                    steer_angle = msg.data[6]
                    
                    rospy.loginfo("CAN Feedback: Actual RPM=%d, Target RPM=%d, Duty=%d%%, Steer=0x%02X",
                                  self.actual_rpm, received_target_rpm, 
                                  self.current_duty, steer_angle)
                    
                    # ROS 토픽으로 발행
                    status_msg = Float32MultiArray()
                    status_msg.data = [cmd_id, self.current_duty, 
                                      self.actual_rpm & 0xFF, (self.actual_rpm >> 8) & 0xFF,
                                      received_target_rpm & 0xFF, (received_target_rpm >> 8) & 0xFF,
                                      steer_angle, 0]
                    self.status_pub.publish(status_msg)
                    
                    # 디버그 정보 발행
                    debug_msg = Float32MultiArray()
                    debug_msg.data = [float(self.target_rpm), float(self.actual_rpm), 
                                     float(self.current_duty), float(steer_angle)]
                    self.debug_pub.publish(debug_msg)
                    
            except can.CanError:
                continue
            except Exception as e:
                rospy.logerr("Error receiving CAN message: %s", str(e))

    def shutdown(self):
        """노드 종료 시 정리"""
        self.is_running = False
        rospy.loginfo("Shutting down CAN Controller ...")
        
        # 정지 명령 전송
        try:
            stop_data = [0x00, 0x00, 0x00, 0xAA, map_steering_angle(90), 0, 0, 0]
            stop_msg = can.Message(arbitration_id=0x001, data=stop_data, is_extended_id=False)
            self.bus.send(stop_msg, timeout=0.1)
        except:
            pass
        
        try:
            if hasattr(self, 'bus') and self.bus is not None:
                self.bus.shutdown()
        except Exception as e:
            rospy.logwarn("CAN bus shutdown failed: %s", str(e))
        rospy.loginfo("CAN Controller shutdown complete")

def main():
    try:
        controller = CANController()
        if controller.bus is not None:
            rospy.on_shutdown(controller.shutdown)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

