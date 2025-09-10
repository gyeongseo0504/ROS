#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import can
import threading
import time
import subprocess
import math

from std_msgs.msg import Bool, Int16, Int32, Float32, String
from geometry_msgs.msg import Twist, Pose2D, Vector3
from sensor_msgs.msg import NavSatFix

class IntegratedCANController:
    def __init__(self):
        rospy.init_node('integrated_can_controller', anonymous=True)

        # ★ 조향 매핑 테이블 (C++ 코드와 동일)
        self.STEER_HEX_LIST = [
            0x00, 0x06, 0x0C, 0x12, 0x18, 0x1E, 0x24, 0x2A, 0x30, 0x36, 0x3C,
            0x42, 0x48, 0x4E, 0x54, 0x5A, 0x60, 0x66, 0x6C, 0x72, 0x78, 0x7E,
            0x84, 0x8A, 0x90, 0x96, 0x9C, 0xA2, 0xA8, 0xAE, 0xB4, 0xBA, 0xC0,
            0xC6, 0xCC, 0xD2, 0xD8, 0xDE, 0xE4, 0xEA, 0xF0, 0xF6, 0xF6
        ]
        self.RIGHT_MAX = -24   # 우회전 한계
        self.LEFT_MAX  = 25   # 좌회전 한계

        # 차량 제어 상태
        self.steering_angle_cmd = 0       # [-24, +25] deg
        self.motor_speed_cmd    = 0.0     # 0~16000
        self.current_rpm_fb     = 0
        self.current_steer_fb   = 90

        # 내부 GPS 추종 파라미터
        self.use_internal_gps_follow = rospy.get_param("~use_internal_gps_follow", True)
        self.kp_yaw_deg     = rospy.get_param("~kp_yaw_deg", 1.0)        # yaw 오차(도) 이득
        self.kp_cte_deg     = rospy.get_param("~kp_cte_deg", 0.0)        # CTE(미터) 이득
        self.speed_cruise   = rospy.get_param("~speed_cruise", 3000.0)   # 기본 속도
        self.goal_reach_dist= rospy.get_param("~goal_reach_dist", 1.5)   # 도달 판정(m)

        # GPS/Datum/Goal/IMU 상태
        self.datum_lat = None
        self.datum_lon = None
        self.datum_yaw = 0.0
        self.cur_lat = None
        self.cur_lon = None
        self.have_fix = False
        self.goal_pose = None             # Pose2D(x=E[m], y=N[m], theta는 선택)
        self.imu_heading_rad = 0.0
        self.have_imu = False
        self.cross_track_error = 0.0

        # CAN 상태
        self.bus = None
        self.can_error_count = 0
        self.emergency_stop = False

        # ROS I/O
        self.setup_subscribers()
        self.setup_publishers()

        # CAN 초기화
        self.init_can_interface()

        # 수신 스레드 시작
        self.can_thread = threading.Thread(target=self.can_receiver_thread, daemon=True)
        self.can_thread.start()

        # 50Hz 제어 루프
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)

        rospy.loginfo("통합 CAN 제어 노드 시작됨")

    # -------------------- ROS I/O --------------------
    def setup_subscribers(self):
        # 기존 호환 입력
        rospy.Subscriber('/Car_Control_Cmd/speed_Float32', Float32, self.speed_callback)
        rospy.Subscriber('/Car_Control_Cmd/Target_Angle', Float32, self.target_angle_callback)
        rospy.Subscriber('/Car_Control_Cmd/steerAngle_Int16', Int16, self.steer_int16_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_stop_callback)
        rospy.Subscriber('/imu/heading_angle_radian', Float32, self.imu_callback)

        # GPS 추종용 입력 (정식 타입으로 구독)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_fix_callback)
        rospy.Subscriber('/gps/datum', Vector3, self.gps_datum_callback)      # x=lat, y=lon, z=yaw(rad 또는 deg는 퍼블리셔 기준)
        rospy.Subscriber('/wp/target_goal', Pose2D, self.wp_target_goal_callback)

        # 선택: CTE 보정
        rospy.Subscriber('/cross_track_error', Float32, self.cross_track_error_callback)

    def setup_publishers(self):
        self.rpm_pub    = rospy.Publisher('/current_rpm', Int32, queue_size=1)
        self.steer_pub  = rospy.Publisher('/current_steer_angle', Int16, queue_size=1)
        self.status_pub = rospy.Publisher('/vehicle/status', String, queue_size=1)

    # -------------------- 콜백 --------------------
    def speed_callback(self, msg: Float32):
        self.motor_speed_cmd = max(-16000.0, min(16000.0, float(msg.data)))

    def target_angle_callback(self, msg: Float32):
        angle = int(round(msg.data))
        self.steering_angle_cmd = max(self.RIGHT_MAX, min(self.LEFT_MAX, angle))
        rospy.loginfo(f"조향 명령 (Float32): {msg.data} -> {self.steering_angle_cmd}")

    def steer_int16_callback(self, msg: Int16):
        self.steering_angle_cmd = max(self.RIGHT_MAX, min(self.LEFT_MAX, int(msg.data)))
        rospy.loginfo(f"조향 명령 (Int16): {msg.data} -> {self.steering_angle_cmd}")

    def cmd_vel_callback(self, msg: Twist):
        self.steering_angle_cmd = max(self.RIGHT_MAX, min(self.LEFT_MAX, int(msg.angular.z)))
        self.motor_speed_cmd = float(msg.linear.x)

    def emergency_stop_callback(self, msg: Bool):
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.steering_angle_cmd = 0
            self.motor_speed_cmd = 0.0
            rospy.logwarn("비상정지 활성화")

    def imu_callback(self, msg: Float32):
        self.imu_heading_rad = float(msg.data)
        self.have_imu = True

    def gps_fix_callback(self, msg: NavSatFix):
        self.cur_lat = float(msg.latitude)
        self.cur_lon = float(msg.longitude)
        self.have_fix = True

    def gps_datum_callback(self, msg: Vector3):
        self.datum_lat = float(msg.x)
        self.datum_lon = float(msg.y)
        self.datum_yaw = float(msg.z)   # 사용 여부는 상황에 따라
        rospy.loginfo_once("Datum 수신 완료")

    def wp_target_goal_callback(self, msg: Pose2D):
        # 목표점을 datum 기준 EN(m) 좌표로 받는다고 가정
        self.goal_pose = Pose2D(msg.x, msg.y, msg.theta)

    def cross_track_error_callback(self, msg: Float32):
        self.cross_track_error = float(msg.data)

    # -------------------- CAN 셋업 --------------------
    def init_can_interface(self):
        try:
            if self.bus is not None:
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
                time.sleep(0.5)

            self.setup_can_interface()
            self.bus = can.interface.Bus(channel='can0', interface='socketcan', can_filters=None)
            rospy.loginfo("CAN 버스 연결 성공: can0")
            self.can_error_count = 0
            return True
        except Exception as e:
            rospy.logerr(f"CAN 초기화 실패: {e}")
            self.bus = None
            return False

    def setup_can_interface(self):
        try:
            rospy.loginfo("CAN 인터페이스 설정 중...")
            subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'down'], check=False, capture_output=True)
            subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'up', 'type', 'can', 'bitrate', '500000'],
                           check=True, capture_output=True, text=True)
            subprocess.run(['sudo', 'ip', 'link', 'set', 'can0', 'txqueuelen', '1000'],
                           check=False, capture_output=True)
            rospy.loginfo("CAN 인터페이스 설정 완료")
            return True
        except Exception as e:
            rospy.logerr(f"CAN 인터페이스 설정 실패: {e}")
            return False

    # -------------------- 유틸 --------------------
    def steer_cmd_to_index(self, steer_cmd: int) -> int:
        steer_cmd = max(self.RIGHT_MAX, min(self.LEFT_MAX, int(steer_cmd)))
        in_min, in_max = self.RIGHT_MAX, self.LEFT_MAX
        out_min, out_max = 0, len(self.STEER_HEX_LIST) - 1
        idx_f = ((steer_cmd - in_min) * (out_max - out_min)) / float(in_max - in_min) + out_min
        idx = int(round(idx_f))
        return max(0, min(len(self.STEER_HEX_LIST) - 1, idx))

    def create_can_data(self):
        """
        C++ 노드와 동일 포맷(8바이트):
        [0]=Ctrl(방향), [1]=RPM_L, [2]=RPM_H, [3]=0xAA, [4]=SteerHex, [5]=0x00, [6]=0x00, [7]=0x00
        """
        frame_data = [0x00] * 8

        # 속도 방향
        if self.motor_speed_cmd > 0:   frame_data[0] = 0x82
        elif self.motor_speed_cmd < 0: frame_data[0] = 0x02
        else:                           frame_data[0] = 0x00

        # RPM (리틀엔디언)
        rpm_val = int(abs(self.motor_speed_cmd))
        rpm_val = max(0, min(16000, rpm_val))
        frame_data[1] = rpm_val & 0xFF
        frame_data[2] = (rpm_val >> 8) & 0xFF

        # 조향 앞 고정 바이트
        frame_data[3] = 0xAA

        # 조향 HEX 테이블 적용
        idx = self.steer_cmd_to_index(self.steering_angle_cmd)
        frame_data[4] = self.STEER_HEX_LIST[idx]

        # 패딩
        frame_data[5] = 0x00
        frame_data[6] = 0x00
        frame_data[7] = 0x00

        rospy.loginfo_throttle(0.5,
            f"[CAN TX] steer_cmd={self.steering_angle_cmd} idx={idx} hex=0x{frame_data[4]:02X} rpm={rpm_val}")
        return frame_data

    # -------------------- GPS 추종 --------------------
    def maybe_update_from_gps(self):
        """
        내부 GPS 추종 모드일 때:
         - datum(lat,lon) 기준 로컬 EN(m)으로 현재 위치 변환
         - 목표점까지 방위각 → 현재 헤딩과의 오차 → 조향각 계산
        """
        if not self.use_internal_gps_follow:
            return
        if not (self.have_fix and self.goal_pose and self.datum_lat is not None and self.datum_lon is not None):
            return

        # 1) 위경도 → datum 기준 EN(m) 근사 (equirectangular)
        lat0 = math.radians(self.datum_lat)
        dlat = math.radians(self.cur_lat - self.datum_lat)
        dlon = math.radians(self.cur_lon - self.datum_lon)
        R = 6378137.0
        x_e = R * dlon * math.cos(lat0)   # East
        y_n = R * dlat                    # North

        # 2) 목표점과의 벡터
        dx = self.goal_pose.x - x_e
        dy = self.goal_pose.y - y_n
        dist = math.hypot(dx, dy)

        # 3) 도달 판정
        if dist <= self.goal_reach_dist:
            self.motor_speed_cmd = 0.0
            rospy.loginfo_throttle(1.0, f"[GPS-FOLLOW] 목표 도달 dist={dist:.2f}m → 정지")
            return

        # 4) 목표 방위각(라디안)
        yaw_target_rad = math.atan2(dy, dx)
        yaw_target_deg = math.degrees(yaw_target_rad)

        # 5) 현재 헤딩(도)
        if self.have_imu:
            yaw_cur_deg = math.degrees(self.imu_heading_rad)
        else:
            yaw_cur_deg = yaw_target_deg  # IMU 없으면 보수적으로 타깃에 정렬

        # 6) 각도 오차 [-180,180]
        err_yaw = yaw_target_deg - yaw_cur_deg
        while err_yaw > 180:  err_yaw -= 360
        while err_yaw < -180: err_yaw += 360

        # 7) (옵션) CTE 보정 추가
        steer_cmd = self.kp_yaw_deg * err_yaw + self.kp_cte_deg * self.cross_track_error

        # 8) 제한 및 적용
        steer_cmd = int(round(max(self.RIGHT_MAX, min(self.LEFT_MAX, steer_cmd))))
        self.steering_angle_cmd = steer_cmd

        # 9) 속도(크루즈)
        if self.motor_speed_cmd == 0.0:
            self.motor_speed_cmd = self.speed_cruise

        rospy.loginfo_throttle(
            0.5,
            f"[GPS-FOLLOW] dist={dist:.2f}m target={yaw_target_deg:.1f}° cur={yaw_cur_deg:.1f}° "
            f"err={err_yaw:.1f}° cte={self.cross_track_error:.2f} → steer={self.steering_angle_cmd}"
        )

    # -------------------- 루프/전송/수신 --------------------
    def control_loop(self, event):
        try:
            # 내부 GPS 추종 갱신
            self.maybe_update_from_gps()

            if self.bus is None:
                rospy.logwarn_throttle(5, "CAN 버스 연결되지 않음, 재연결 시도")
                self.init_can_interface()
                return

            # 비상정지 프레임 or 일반 프레임
            if self.emergency_stop:
                can_data = [0x00, 0x00, 0x00, 0xAA, self.STEER_HEX_LIST[21], 0x00, 0x00, 0x00]  # 직진 정지
            else:
                can_data = self.create_can_data()

            self.send_can_message(can_data)
        except Exception as e:
            rospy.logerr(f"제어 루프 오류: {e}")
            self.init_can_interface()

    def send_can_message(self, data):
        if self.bus is None:
            return False

        max_retries = 3
        for attempt in range(max_retries):
            try:
                msg = can.Message(arbitration_id=0x01, data=data, is_extended_id=False)
                self.bus.send(msg, timeout=0.1)
                return True
            except can.CanError as e:
                if "No buffer space available" in str(e) or "Bad file descriptor" in str(e):
                    rospy.logwarn("CAN 전송 오류, 재연결 시도")
                    if self.init_can_interface():
                        continue
                    return False
                time.sleep(0.05)
            except Exception as e:
                rospy.logerr(f"예상치 못한 CAN 오류: {e}")
                self.init_can_interface()
                return False

        rospy.logerr("CAN 전송 실패(재시도 초과)")
        return False

    def can_receiver_thread(self):
        rospy.loginfo("CAN 수신 스레드 시작")
        while not rospy.is_shutdown():
            try:
                if self.bus is not None:
                    message = self.bus.recv(timeout=0.1)
                    if message is not None:
                        self.process_received_can(message)
                else:
                    time.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"CAN 수신 오류: {e}")
                if "Bad file descriptor" in str(e) or "negative integer" in str(e):
                    rospy.logwarn("CAN 재연결 시도...")
                    self.init_can_interface()
                time.sleep(0.5)

    def process_received_can(self, message):
        can_id = message.arbitration_id
        data = message.data
        try:
            if can_id == 0x302:
                self.process_vehicle_feedback(data)
        except Exception as e:
            rospy.logerr(f"CAN 메시지 처리 오류: {e}")

    def process_vehicle_feedback(self, data):
        if len(data) >= 8:
            try:
                self.current_rpm_fb = data[1] | (data[2] << 8)
                self.current_steer_fb = data[4]
                self.rpm_pub.publish(Int32(self.current_rpm_fb))
                self.steer_pub.publish(Int16(self.current_steer_fb))
                self.status_pub.publish(String(f"RPM:{self.current_rpm_fb}, Steer:0x{self.current_steer_fb:02X}"))
            except Exception as e:
                rospy.logerr(f"피드백 데이터 파싱 오류: {e}")

    def shutdown(self):
        rospy.loginfo("통합 CAN 제어 노드 종료")
        try:
            if self.bus is not None:
                stop_data = [0x00, 0x00, 0x00, 0xAA, self.STEER_HEX_LIST[21], 0x00, 0x00, 0x00]
                self.send_can_message(stop_data)
                self.bus.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    try:
        controller = IntegratedCANController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("노드 중단됨")
    except Exception as e:
        rospy.logerr(f"노드 실행 오류: {e}")
