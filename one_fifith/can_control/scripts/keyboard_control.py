#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

msg = """
CAN Motor Control (PID Mode with Reverse)
---------------------------
Motor Specs:
- Drive Motor: 24V, Max Duty 80% (-16000 ~ +16000 RPM)
- Steer Motor: 12-15V, Max Duty 20%

Moving around:
        ↑
        w
   ←a   s   d→
        ↓

w/s : forward/reverse (increase/decrease RPM by 500)
shift+w/s : fine control (increase/decrease RPM by 100)
a/d : decrease/increase steering by 1
space : stop (RPM = 0)
r : reset PID integral term
x : toggle forward/reverse

CTRL-C to quit
---------------------------
Current Status:
"""

def getKey():
    """키보드 입력 받기"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        
        # Publisher 설정
        self.pub = rospy.Publisher('/can_control_cmd', 
                                  Float32MultiArray, 
                                  queue_size=10)
        
        # Subscriber 설정 (피드백 수신)
        self.sub = rospy.Subscriber('/can_status',
                                   Float32MultiArray,
                                   self.status_callback)
        
        # 초기값 설정
        self.target_rpm = 0.0
        self.actual_rpm = 0.0
        self.steering = 90.0  # 90도가 정중앙
        self.current_duty = 0
        
        # RPM과 조향 제한값 (맵핑 테이블 구간과 완벽히 일치)
        self.max_rpm = 16000.0   # 최대 전진 RPM
        self.min_rpm = -16000.0  # 최대 후진 RPM
        self.max_steering = 111.0  # 오른쪽 최대 (수정)
        self.min_steering = 69.0   # 왼쪽 최대 (수정)
        
        # 증가/감소 단위
        self.rpm_step_large = 500.0  # 큰 단위
        self.rpm_step_small = 100.0  # 작은 단위
        self.steering_step = 1.0
        
        rospy.loginfo("Keyboard Control initialized (PID Mode with Reverse)")
        print(msg)
        self.print_status()
    
    def status_callback(self, msg):
        """CAN 상태 피드백 수신"""
        if len(msg.data) >= 6:
            # 실제 RPM 파싱 (Little-endian, signed)
            rpm_bytes = (int(msg.data[3]) << 8) | int(msg.data[2])
            # 부호 있는 16비트로 변환
            if rpm_bytes > 32767:
                self.actual_rpm = rpm_bytes - 65536
            else:
                self.actual_rpm = rpm_bytes
            self.current_duty = int(msg.data[1])
    
    def print_status(self):
        """현재 상태 출력"""
        direction = "FWD" if self.target_rpm >= 0 else "REV"
        duty_warning = " ⚠️" if self.current_duty >= 80 else ""
        print("\r[%s] Target: %6.0f RPM | Actual: %6.0f RPM | Duty: %3d%%%s | Steer: %3.0f°" % 
              (direction, self.target_rpm, self.actual_rpm, self.current_duty, duty_warning, self.steering), end='')
        sys.stdout.flush()
    
    def publish_command(self):
        """제어 명령 발행"""
        msg = Float32MultiArray()
        msg.data = [self.target_rpm, self.steering]
        self.pub.publish(msg)
    
    def run(self):
        """메인 실행 루프"""
        rate = rospy.Rate(100)  # 50Hz
        
        try:
            while not rospy.is_shutdown():
                key = getKey()
                
                # w키 - 전진 방향 증가
                if key == 'w':  # 소문자 w - 큰 단위
                    if self.target_rpm >= 0:
                        self.target_rpm = min(self.target_rpm + self.rpm_step_large, self.max_rpm)
                    else:
                        self.target_rpm = min(self.target_rpm + self.rpm_step_large, 0)
                    self.print_status()
                    
                elif key == 'W':  # 대문자 W - 작은 단위
                    if self.target_rpm >= 0:
                        self.target_rpm = min(self.target_rpm + self.rpm_step_small, self.max_rpm)
                    else:
                        self.target_rpm = min(self.target_rpm + self.rpm_step_small, 0)
                    self.print_status()
                    
                # s키 - 후진 방향 증가 (또는 전진 감소)
                elif key == 's':  # 소문자 s - 큰 단위
                    if self.target_rpm <= 0:
                        self.target_rpm = max(self.target_rpm - self.rpm_step_large, self.min_rpm)
                    else:
                        self.target_rpm = max(self.target_rpm - self.rpm_step_large, 0)
                    self.print_status()
                    
                elif key == 'S':  # 대문자 S - 작은 단위
                    if self.target_rpm <= 0:
                        self.target_rpm = max(self.target_rpm - self.rpm_step_small, self.min_rpm)
                    else:
                        self.target_rpm = max(self.target_rpm - self.rpm_step_small, 0)
                    self.print_status()
                    
                elif key == 'a' or key == '\x1b[D':  # 왼쪽 화살표
                    self.steering = max(self.steering - self.steering_step, self.min_steering)  # 69도로 제한
                    self.print_status()
                    
                elif key == 'd' or key == '\x1b[C':  # 오른쪽 화살표
                    self.steering = min(self.steering + self.steering_step, self.max_steering)  # 111도로 제한
                    self.print_status()
                    
                elif key == ' ':  # 스페이스바 - 정지
                    self.target_rpm = 0.0
                    self.print_status()
                    
                elif key == 'x' or key == 'X':  # 전진/후진 토글
                    self.target_rpm = -self.target_rpm
                    rospy.loginfo("\n[Direction Toggle] Changed to %s", 
                                  "Forward" if self.target_rpm >= 0 else "Reverse")
                    self.print_status()
                    
                elif key == 'r' or key == 'R':  # PID 리셋
                    rospy.loginfo("\n[PID Reset] Integral term will be reset")
                    self.print_status()
                    
                elif key == '\x03':  # CTRL-C
                    break
                
                # 명령 발행
                self.publish_command()
                self.print_status()  # 지속적으로 상태 업데이트
                rate.sleep()
                
        except Exception as e:
            print("\nError: %s" % str(e))
        finally:
            # 종료 시 정지 명령 전송
            self.target_rpm = 0.0
            self.publish_command()
            print("\n\nKeyboard control terminated")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

