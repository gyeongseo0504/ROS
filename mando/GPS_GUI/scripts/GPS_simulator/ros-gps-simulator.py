# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gps_test.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from __future__ import print_function

import math
import os,sys
import rospy
import time
import sensor_msgs.msg 
import geometry_msgs.msg
import tf

print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import QuaternionStamped

from PyQt5.QtCore import *
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from pyqtgraph import PlotWidget
import pyqtgraph as pg


import numpy as np
import math
import utm

global gps_datum_data
gps_datum_data=[0.0,0.0,0.0]
global gps_fix_data
gps_fix_data=[0.0,0.0,0.0]

form_class = uic.loadUiType('gps_simulator.ui')[0]

gps_utm    = np.zeros(2)
gps1_utm   = np.zeros(2)
gps2_utm   = np.zeros(2)

gps1_lla   = np.zeros(2)
gps2_lla   = np.zeros(2)

offset_gps = 0.5
base_move  = 0.1




def gps_fix_pub(self):
    global gps_fix_data    
    pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
    msg = NavSatFix()
    
    msg.header.stamp = rospy.Time.now()   
    msg.header.frame_id = 'gps'
    msg.latitude = gps_fix_data[0]
    msg.longitude = gps_fix_data[1]
    msg.altitude = 0
    msg.status.status = 2
    msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    msg.position_covariance_type = 0
    
    pub.publish(msg)


	
    
def gps1_fix_pub(self):
    global gps1_lla
    pub = rospy.Publisher("/gps1/fix", NavSatFix, queue_size=10)
    msg = NavSatFix()
    
    msg.header.stamp = rospy.Time.now()   
    msg.header.frame_id = 'gps'
    msg.latitude = gps1_lla[0]
    msg.longitude = gps1_lla[1]
    msg.altitude = 0
    msg.status.status = 2
    msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
    msg.position_covariance_type = 0
    
    pub.publish(msg)
    
def gps2_fix_pub(self):
	global gps2_lla
	pub = rospy.Publisher("/gps2/fix", NavSatFix, queue_size=10)
	msg = NavSatFix()
    
	msg.header.stamp = rospy.Time.now()   
	msg.header.frame_id = 'gps'
	msg.latitude  = gps2_lla[0]
	msg.longitude = gps2_lla[1]
	msg.altitude  = 0
	msg.status.status = 2
	msg.position_covariance = (0, 0, 0, 0, 0, 0, 0, 0, 0)
	msg.position_covariance_type = 0
    
	pub.publish(msg)
    
def gps_heading_pub(self):
	global gps_fix_data
	pub = rospy.Publisher("heading", QuaternionStamped, queue_size=10)
    
    
	msg = QuaternionStamped()
	msg.header.stamp = rospy.Time.now()   
	msg.header.frame_id = 'gps'
    
	quaternion = tf.transformations.quaternion_from_euler(0, 0, gps_fix_data[2]/180.0*math.pi)
	msg.quaternion.x = quaternion[0]
	msg.quaternion.y = quaternion[1]
	msg.quaternion.z = quaternion[2]
	msg.quaternion.w = quaternion[3]
    
	pub.publish(msg)    
    

class Thread_GPS_Publish(QThread):
	global auto_replay_flag 
	def __init__(self, parent): 
		super(Thread_GPS_Publish,self).__init__(parent)    
		self.parent = parent #self.parent를 사용하여 WindowClass 위젯을 제어할 수 있다.		
		print("GPS_Publish")
		
	def run(self):
		
		while(True):
			print("GPS")
			gps1_fix_pub(self.parent)	
			gps2_fix_pub(self.parent)	
			gps_heading_pub(self.parent)
			time.sleep(0.5)
		
	
	'''		
	def stop(self):
		if auto_replay_flag  == 0:
			print("Thread_Waypoints_Publish quit")
			self.quit()
	'''		
	
class WindowClass(QDialog, form_class):
	def __init__(self):
		
		super(QDialog,self).__init__()
		
		rospy.init_node('ros_gps_sim', anonymous=True)
		
		self.setupUi(self)
		
		self.setWindowTitle("GPS Simulator")
		
		self.thread = Thread_GPS_Publish(self)
		self.thread.start() 
		
		#self.textEdit_gps_latitude.setText("37.41856")   
		#self.textEdit_gps_longitude.setText("128.753342")
		
		self.textEdit_gps_latitude.setText("37.419851558798534")   
		self.textEdit_gps_longitude.setText("127.12536248103291") 
		
		
		self.textEdit_gps_yaw.setText("0")
		self.textEdit_gps_base_move.setText("0.1")
        
        
		self.pushButton_datum.clicked.connect(self.datum_button_Clicked)
		self.pushButton_move_up.clicked.connect(self.gps_move_up_button_Clicked)
		self.pushButton_move_down.clicked.connect(self.gps_move_down_button_Clicked)
		self.pushButton_move_right.clicked.connect(self.gps_move_right_button_Clicked)
		self.pushButton_move_left.clicked.connect(self.gps_move_left_button_Clicked)
		
       
		self.horizontalSlider.setRange(-180,180)
		self.horizontalSlider.setTickInterval(10)
		self.horizontalSlider.valueChanged.connect(self.slider_value_changed)
		
	def lla_to_utm(self):
		print("base gps : lla to utm")
		global offset_gps
		global gps1_utm
		global gps2_utm
		
		angle1 = gps_fix_data[2] 
		angle2 = angle1 + 180.0
		print(angle1, angle2)  #utm_e 를 기준으로 각도
		coordinate = utm.from_latlon(gps_fix_data[0], gps_fix_data[1])
		gps_utm[0] = coordinate[0]
		gps_utm[1] = coordinate[1]
		
		#print( math.cos( math.radians(angle1) ) , math.cos( math.radians(angle2) ) )
		
		utm1_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle1) )
		utm1_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle1) )
		
		utm2_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle2) )
		utm2_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle2) )
		
		gps1_utm[0] = utm1_x
		gps1_utm[1] = utm1_y
		
		gps2_utm[0] = utm2_x
		gps2_utm[1] = utm2_y
		
		print("Front :", gps1_utm[0], gps1_utm[1])
		print("Rear  :",gps2_utm[0], gps2_utm[1])
		
		#print("bass gps utm ", gps_utm[0], gps_utm[1])
		return
		
	def utm_to_lla(self):
		print("utm to lla")
		global offset_gps
		global gps_fix_data
		global gps1_lla
		global gps2_lla
		
		angle1 = gps_fix_data[2]
		angle2 = angle1 +180.0
		
		gos_lla_coordinate = utm.to_latlon(gps_utm[0], gps_utm[1],52,'U')
		print("GPS  :", gos_lla_coordinate[0], gos_lla_coordinate[1])
		
		gps_fix_data[0] = gos_lla_coordinate[0]
		gps_fix_data[1] = gos_lla_coordinate[1]
		
		coordinate = utm.from_latlon(gps_fix_data[0], gps_fix_data[1])
		gps_utm[0] = coordinate[0]
		gps_utm[1] = coordinate[1]
		
		#print( math.cos( math.radians(angle1) ) , math.cos( math.radians(angle2) ) )
		
		utm1_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle1) )
		utm1_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle1) )
		
		utm2_x = gps_utm[0] - offset_gps * math.sin( math.radians(angle2) )
		utm2_y = gps_utm[1] + offset_gps * math.cos( math.radians(angle2) )
		
		gps1_utm[0] = utm1_x
		gps1_utm[1] = utm1_y
		
		gps1_fix_data= utm.to_latlon(gps1_utm[0], gps1_utm[1],52,'U')
		gps1_lla = gps1_fix_data
		print("GPS1 :", gps1_fix_data[0],gps1_fix_data[1])
		
		gps2_utm[0] = utm2_x
		gps2_utm[1] = utm2_y
		
		gps2_fix_data= utm.to_latlon(gps2_utm[0], gps2_utm[1],52,'U')
		gps2_lla = gps2_fix_data
		print("GPS2 :", gps2_fix_data[0],gps2_fix_data[1])
		
		return			
    
	def subGPSdatumCallback(self,gps_datum_msg):
		#print(gps_datum_msg.x, gps_datum_msg.y ,gps_datum_msg.z)
		global gps_datum_data
		gps_datum_data=[gps_datum_msg.x , gps_datum_msg.y ,gps_datum_msg.z]
		
		#print(gps_datum_data)
		
	def datum_button_Clicked(self):
		global gps_datum_data        
		global gps_fix_data
		#print(gps_datum_data)				
		self.textEdit_datum_latitude.setText(str(gps_datum_data[0]))
		self.textEdit_datum_longitude.setText(str(gps_datum_data[1]))
		self.textEdit_datum_yaw.setText(str(gps_datum_data[2]))
		gps_fix_data = [gps_datum_data[0],gps_datum_data[1],gps_datum_data[2]]
        
	def gps_move_up_button_Clicked(self):
		#print("test")
		global gps_fix_data
		global base_move 				
		        
		txt = self.textEdit_gps_latitude.toPlainText()
		gps_fix_data[0] = float(txt);
		txt = self.textEdit_gps_longitude.toPlainText()
		gps_fix_data[1] = float(txt);
		txt = self.textEdit_gps_yaw.toPlainText()		
		gps_fix_data[2] = float(txt);
		
		self.lla_to_utm()
		gps_utm[1] = gps_utm[1]+ base_move
		self.utm_to_lla()		
		
		#gps_fix_data[0] =  gps_fix_data[0] + 0.000001  #north
		
		self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
		self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
			
		gps_fix_pub(self)         
		gps1_fix_pub(self)         
		gps2_fix_pub(self)  

	def gps_move_down_button_Clicked(self):
		#print("test")
		global gps_fix_data

		txt = self.textEdit_gps_latitude.toPlainText()
		gps_fix_data[0] = float(txt);
		txt = self.textEdit_gps_longitude.toPlainText()
		gps_fix_data[1] = float(txt);
		txt = self.textEdit_gps_yaw.toPlainText()
		gps_fix_data[2] = float(txt);
		
		self.lla_to_utm()
		gps_utm[1] = gps_utm[1] - base_move
		self.utm_to_lla()
        
		self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
		self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
		
		gps_fix_pub(self)         
		gps1_fix_pub(self)         
		gps2_fix_pub(self)  
        
	def gps_move_right_button_Clicked(self):
		#print("test")
		global gps_fix_data
        
		txt = self.textEdit_gps_latitude.toPlainText()
		gps_fix_data[0] = float(txt);
		txt = self.textEdit_gps_longitude.toPlainText()
		gps_fix_data[1] = float(txt);
		txt = self.textEdit_gps_yaw.toPlainText()
		gps_fix_data[2] = float(txt);
		
        
		self.lla_to_utm()
		gps_utm[0] = gps_utm[0] + base_move
		self.utm_to_lla()   
        
		self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
		self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
		
		gps_fix_pub(self)         
		gps1_fix_pub(self)         
		gps2_fix_pub(self)   
    
	def gps_move_left_button_Clicked(self):
		#print("test")
		global gps_fix_data
        
		txt = self.textEdit_gps_latitude.toPlainText()
		gps_fix_data[0] = float(txt);
		txt = self.textEdit_gps_longitude.toPlainText()
		gps_fix_data[1] = float(txt);
		txt = self.textEdit_gps_yaw.toPlainText()
		gps_fix_data[2] = float(txt);
		
		self.lla_to_utm()
		gps_utm[0] = gps_utm[0] - base_move
		self.utm_to_lla()
       
		self.textEdit_gps_latitude.setText(str(gps_fix_data[0]))
		self.textEdit_gps_longitude.setText(str(gps_fix_data[1]))
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
				
		gps_fix_pub(self)         
		gps1_fix_pub(self)         
		gps2_fix_pub(self)         
		return
    
    
	def slider_value_changed(self,value):
		global gps_fix_data
		gps_fix_data[2] = value
		self.textEdit_gps_yaw.setText(str(gps_fix_data[2]))
		self.lla_to_utm()
		self.utm_to_lla()
		gps_heading_pub(self)
		gps_fix_pub(self)         
		gps1_fix_pub(self)         
		gps2_fix_pub(self) 		
		#print("slider",value)
		return


if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()

