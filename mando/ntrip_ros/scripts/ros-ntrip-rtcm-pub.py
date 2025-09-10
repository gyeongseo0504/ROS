import sys
import os
import rospy
from rtcm_msgs.msg import Message
import numpy as np
import threading
import signal  # signal 모듈 추가

fifo_name = "my_pipe"

data_old = b'\x41'
data = b'\x41'
data_len = 0
data_len_old = 0

pub = None  # Publisher를 전역으로 이동하여 다른 함수에서 접근할 수 있도록 함
rospy.init_node('ros_ntrip_kkw', anonymous=True)
 
def pub_rtcm(data_buf):
	rmsg = Message()
	pub = rospy.Publisher('/gps/rtcm', Message, queue_size=10)
	
	rmsg.message = data_buf
	rmsg.header.seq += 1
	rmsg.header.stamp = rospy.Time.now()   
	pub.publish(rmsg)
	
	return
	
class ntrip_pip_read(threading.Thread):
    
    def __init__(self):
        super(ntrip_pip_read, self).__init__()
        self._stop_event = threading.Event()  # 스레드 종료 이벤트

    def run(self):
        global fifo_name
        global data_len, data_len_old
        global data, data_old
        while not self._stop_event.is_set():  # 종료 이벤트를 체크
            with open(fifo_name, "rb") as pipe:
                data = pipe.read()
            data_len = len(data)
            if (data!= data_old) and (data_len != 0):
                print(data_len)
                pub_rtcm(data)
            data_len_old = data_len
            data_old        = data

    def stop(self):
        self._stop_event.set()  # 스레드 종료 이벤트 설정

def signal_handler(signal, frame):
    print("Received Ctrl-C. Exiting...")
    ntrip_thread.stop()  # 스레드 종료
    sys.exit(0)

def rtcm_client_pub():
    global data, data_old, fifo_name
    global pub
    
   
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        ntrip_thread = ntrip_pip_read()
        ntrip_thread.start()
        
        # Ctrl-C 시그널 핸들러 등록
        signal.signal(signal.SIGINT, signal_handler)
        
        rtcm_client_pub()
    except rospy.ROSInterruptException:
        pass
