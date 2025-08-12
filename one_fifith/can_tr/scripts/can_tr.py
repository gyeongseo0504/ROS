#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float32, Float64
import can
import time
import errno

class CANCombinedController:
    def __init__(self):
        rospy.init_node('can_combined_controller', anonymous=True)

        # CAN 버스 초기화
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            rospy.loginfo("CAN interface initialized (can0).")
        except Exception as e:
            rospy.logerr(f"CAN interface init failed: {e}")
            self.bus = None

        # 기본 값 설정
        self.CAN_ID = 0x01
        self.TARGET_RPM = rospy.get_param('~target_rpm', 1000)
        self.STEERING_ANGLE = 90  # 초기값 90도 (중립)

        # 상태 변수
        self.current_rpm = self.TARGET_RPM
        self.current_control_id = 0x82  # 기본 전진 명령

        self.vehicle_state = "UNDETECTED"
        self.current_light_status = "unknown"
        self.last_can_time = 0

        # 미인식 상태 관리
        self.undetected_start_time = 0
        self.undetected_duration = 3.0

        # 초록불 대기 타이머
        self.green_wait_start_time = 0

        # 감지 소스
        self.detection_source = ""

        # 스티어링 각도 제어용 변수 (차선 인식 결과)
        self.goal_angle = 0.0

        # 8바이트 값 기록
        self.last_sent_data = [0]*8

        # Subscriber 등록
        rospy.Subscriber('/traffic_light_detection', String, self.traffic_callback)
        rospy.Subscriber('/traffic_light_confidence', Float32, self.confidence_callback)
        rospy.Subscriber('/goal_angle', Float64, self.goal_angle_callback)

        # Publisher
        self.status_pub = rospy.Publisher('/vehicle_status', String, queue_size=10)

        # 초기 CAN 메시지 전송
        self.send_can_message(self.current_control_id, self.current_rpm, 0xAA, self.STEERING_ANGLE)
        rospy.loginfo("CANCombinedController started.")

        # 10Hz 타이머 설정, CAN 주기 전송
        self.timer = rospy.Timer(rospy.Duration(0.1), self.periodic_can_send)

    ##############################
    # CAN 메시지 전송 함수들
    ##############################

    def rpm_to_bytes(self, rpm):
        rpm = max(0, min(16000, int(rpm)))
        rpm_bytes = rpm.to_bytes(2, byteorder='little')
        return rpm_bytes[0], rpm_bytes[1]

    def send_can_message(self, control_id, rpm, steering_id=0xAA, steering_angle=90):
        if self.bus is None:
            rospy.logwarn_throttle(5, "CAN bus not initialized.")
            return
        rpm_l, rpm_h = self.rpm_to_bytes(rpm)
        data = [control_id, rpm_l, rpm_h, steering_id, steering_angle, 0, 0, 0]
        msg = can.Message(arbitration_id=self.CAN_ID, data=data, is_extended_id=False)

        # 8byte 로그: 무조건 보냄!
        rospy.loginfo(f"[CAN TX] data: {[f'0x{x:02X}' for x in data]} (ctrl={control_id:02X}, rpm={rpm}, steering={steering_angle})")

        # 반복적으로 CAN 송신을 시도 (버퍼 풀에 걸려도 대기/재시도)
        max_retry = 10
        for retry in range(max_retry):
            try:
                self.bus.send(msg, timeout=0.1)  # timeout 추가
                return  # 성공하면 함수 종료
            except can.CanError as e:
                # 버퍼 풀(ENOBUFS)일 때만 재시도, 그 외에는 break
                if hasattr(e, 'errno') and e.errno == errno.ENOBUFS:
                    if retry == max_retry-1:
                        rospy.logwarn(f"CAN send failed after retries: No buffer space available")
                    else:
                        time.sleep(0.005)  # 아주 짧게 대기 후 재시도
                        continue
                else:
                    rospy.logwarn(f"CAN send failed: {e}")
                    break

    ##############################
    # Callbacks
    ##############################

    def traffic_callback(self, msg):
        try:
            parts = msg.data.split('|')
            if len(parts) >= 4:
                detection_status = parts[0]
                light_status = parts[1]
                confidence = float(parts[2])
                source = parts[3]

                detected = (detection_status == "detected")
                self.update_traffic_light_status(detected, light_status, confidence, source)
        except Exception as e:
            rospy.logwarn_throttle(5, f"Traffic light message parse error: {e}")

    def confidence_callback(self, msg):
        pass

    def goal_angle_callback(self, msg):
        raw_angle = msg.data
        self.goal_angle = max(-21.0, min(21.0, raw_angle))

    ##############################
    # 신호등 상태 처리
    ##############################

    def update_traffic_light_status(self, detected, status, confidence, source):
        current_time = time.time()

        if not detected:
            if self.undetected_start_time == 0:
                self.undetected_start_time = current_time
            elif (current_time - self.undetected_start_time) >= self.undetected_duration:
                if self.vehicle_state != "UNDETECTED":
                    rospy.loginfo("3초 연속 미인식, UNDETECTED 상태로 전환")
                    self._enter_undetected_state()
        else:
            self.undetected_start_time = 0
            self.current_light_status = status
            self.detection_source = source

            if self.vehicle_state == "UNDETECTED":
                self._handle_signal_detection(status)

            elif self.vehicle_state == "STOPPED_GREEN_WAIT":
                if status in ["traffic_red", "traffic_yellow"]:
                    if status == "traffic_red":
                        self.vehicle_state = "STOPPED_RED"
                        rospy.loginfo("초록불 대기 중 빨간불 감지 - 빨간불 정지 상태")
                    else:
                        self.vehicle_state = "STOPPED_YELLOW"
                        rospy.loginfo("초록불 대기 중 노란불 감지 - 노란불 정지 상태")
                    self.green_wait_start_time = 0

            elif self.vehicle_state in ["STOPPED_RED", "STOPPED_YELLOW"]:
                if status == "traffic_green":
                    self._start_moving()
                    rospy.loginfo("초록불 감지 - 바로 출발!")

            elif self.vehicle_state == "MOVING_AFTER_SIGNAL":
                pass

            if self.vehicle_state == "STOPPED_GREEN_WAIT" and self.green_wait_start_time > 0:
                elapsed = current_time - self.green_wait_start_time
                if elapsed >= 2.0:
                    self._start_moving()
                    self.vehicle_state = "MOVING_AFTER_SIGNAL"
                    rospy.loginfo("2초 대기 완료 - 신호 통과 후 주행 시작!")

        self.publish_status()

    def _handle_signal_detection(self, status):
        if status in ["traffic_red", "traffic_yellow"]:
            self.current_control_id = 0x00
            self.current_rpm = 0
            if status == "traffic_red":
                self.vehicle_state = "STOPPED_RED"
                rospy.loginfo("빨간불 감지 - 정지 후 초록불 대기")
            else:
                self.vehicle_state = "STOPPED_YELLOW"
                rospy.loginfo("노란불 감지 - 정지 후 초록불 대기")
        elif status == "traffic_green":
            self.vehicle_state = "STOPPED_GREEN_WAIT"
            self.green_wait_start_time = time.time()
            self.current_control_id = 0x00
            self.current_rpm = 0
            rospy.loginfo("초록불 감지 - 2초 대기 시작")
        else:
            self._enter_undetected_state()

    def _enter_undetected_state(self):
        previous_state = self.vehicle_state
        self.vehicle_state = "UNDETECTED"
        self.current_light_status = "unknown"
        self.green_wait_start_time = 0
        self.undetected_start_time = 0
        self.current_control_id = 0x82
        self.current_rpm = self.TARGET_RPM
        self.last_sent_data = [0]*8  # 초기화
        rospy.loginfo(f"UNDETECTED 상태 진입 완료 ({previous_state} -> UNDETECTED)")

    def _start_moving(self):
        self.current_control_id = 0x82
        self.current_rpm = self.TARGET_RPM
        self.green_wait_start_time = 0
        self.last_sent_data = [0]*8  # 초기화

    ##############################
    # 주기적 CAN 메시지 전송
    ##############################
    def periodic_can_send(self, event):
        angle_target = int(round(90 + self.goal_angle))
        angle_target = max(69, min(111, angle_target))
        self.send_can_message(self.current_control_id, self.current_rpm, 0xAA, angle_target)

    ##############################
    # 상태 메시지 포맷 (출력)
    ##############################
    def get_status_text(self):
        if self.vehicle_state == "UNDETECTED":
            return f"UNDETECTED - MOVING (RPM: {self.current_rpm})"
        elif self.vehicle_state == "STOPPED_RED":
            return f"STOPPED - RED LIGHT"
        elif self.vehicle_state == "STOPPED_YELLOW":
            return f"STOPPED - YELLOW LIGHT"
        elif self.vehicle_state == "STOPPED_GREEN_WAIT":
            if self.green_wait_start_time > 0:
                elapsed = time.time() - self.green_wait_start_time
                remaining = max(0, 2.0 - elapsed)
                return f"GREEN WAIT ({remaining:.1f}s)"
            return "STOPPED - GREEN"
        elif self.vehicle_state == "MOVING_AFTER_SIGNAL":
            return f"MOVING AFTER SIGNAL (RPM: {self.current_rpm})"
        return "UNKNOWN"

    def publish_status(self):
        status_text = f"{self.vehicle_state}|{self.current_light_status}|{self.get_status_text()}|{self.detection_source}"
        self.status_pub.publish(status_text)

    def shutdown(self):
        self.send_can_message(0x00, 0, 0xAA, self.STEERING_ANGLE)
        rospy.loginfo("Shutting down: vehicle stop command sent.")
        if self.bus:
            self.bus.shutdown()

if __name__ == '__main__':
    try:
        controller = CANCombinedController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("CAN Combined Controller Node Terminated")
        pass
