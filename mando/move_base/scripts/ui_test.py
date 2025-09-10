# -*- coding: euc-kr -*-


from __future__ import print_function

import math
import os,sys
import rospy
import sensor_msgs.msg 
import geometry_msgs.msg
import tf
import actionlib


from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from pyqtgraph import PlotWidget
import pyqtgraph as pg


import numpy as np
import math

form_class = uic.loadUiType('delivery_robot_menu1.ui')[0]

print(form_class) 

X    = np.zeros(100)
Y    = np.zeros(100)
Yaw  = np.zeros(100)
no_line = 1

def movebase_client(goal_id):

	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.stamp = rospy.Time.now()
	# set frame
	goal.target_pose.header.frame_id = 'map'

		# set position
	goal.target_pose.pose.position.x = X[goal_id]
	goal.target_pose.pose.position.y = Y[goal_id]
	goal.target_pose.pose.position.z = 0.0

		# set orientation
	quat = quaternion_from_euler (0, 0,math.radians(Yaw[goal_id]))
	
	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]
		

	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		if client.get_state() == GoalStatus.SUCCEEDED:
			rospy.loginfo('Succeeded to reach  %d-Goal',goal_id)
		else:
			rospy.loginfo('Failed to reach  %d-Goal',goal_id)

		
		return client.get_result()


class WindowClass(QDialog, form_class):
	def __init__(self):
		rospy.init_node('ROS_robot_gui', anonymous=True)		
		
		super(QDialog,self).__init__()
		
		self.setupUi(self)
		#self.setBackground('w')
		self.graphWidget.plot([1,2],[3,2])
		self.read_waypoint_file('//home//amap//robot_catkin_ws//src//kobuki_nav//scripts//waypoint.dat')
				
		self.pushButton_0.clicked.connect(self.readfileFunction)
		self.pushButton_1.clicked.connect(self.Table1_move)
		self.pushButton_2.clicked.connect(self.Table2_move)
		self.pushButton_3.clicked.connect(self.Table3_move)
		self.pushButton_4.clicked.connect(self.Table4_move)
		self.pushButton_5.clicked.connect(self.Table5_move)
		self.pushButton_6.clicked.connect(self.Table6_move)
		self.pushButton_7.clicked.connect(self.Table7_move)
		self.pushButton_8.clicked.connect(self.Table8_move)
		self.pushButton_9.clicked.connect(self.Table9_move)
		self.pushButton_10.clicked.connect(self.Table10_move)
		self.pushButton_Home.clicked.connect(self.Home_move)
		self.pushButton_Charge.clicked.connect(self.Charge_move)
    
    
	def sliderMoved(self):
		print("Dial value = %i" % (self.dial.value()))
            
	def readfileFunction(self):
		print("btn_1 Clicked")
		fname = QFileDialog.getOpenFileName(self, 'Open file', './')
		
		if fname[0]:
			print(fname)
			f = open(fname[0], 'r')
			read_lines= f.readlines()
			no_line = len(read_lines)
			read_lines = list(map(lambda s: s.strip(), read_lines))
			self.waypoint_tableWidget.setRowCount(no_line)
			#print(read_lines)
			for i in range(no_line):
				print(read_lines[i])
				Data = read_lines[i].split()
				X[i]   = float(Data[0])
				Y[i]   = float(Data[1])
				Yaw[i] = float(Data[2])
				self.waypoint_tableWidget.setItem(i,0,QTableWidgetItem(Data[0]))
				self.waypoint_tableWidget.item(i, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
				self.waypoint_tableWidget.setItem(i,1,QTableWidgetItem(Data[1]))
				self.waypoint_tableWidget.item(i, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
				self.waypoint_tableWidget.setItem(i,2,QTableWidgetItem(Data[2]))
				self.waypoint_tableWidget.item(i, 2).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
			f.close()
			
			return
	def read_waypoint_file(self,fname):
		f = open(fname, 'r')
		
		read_lines= f.readlines()
		no_line = len(read_lines)
		read_lines = list(map(lambda s: s.strip(), read_lines))
		self.waypoint_tableWidget.setRowCount(no_line)
		#print(read_lines)
		for i in range(no_line):
			print(read_lines[i])
			Data = read_lines[i].split()
			X[i]   = float(Data[0])
			Y[i]   = float(Data[1])
			Yaw[i] = float(Data[2])
		
			self.waypoint_tableWidget.setItem(i,0,QTableWidgetItem(Data[0]))
			self.waypoint_tableWidget.item(i, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
			self.waypoint_tableWidget.setItem(i,1,QTableWidgetItem(Data[1]))
			self.waypoint_tableWidget.item(i, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
			self.waypoint_tableWidget.setItem(i,2,QTableWidgetItem(Data[2]))
			self.waypoint_tableWidget.item(i, 2).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
		f.close()
	
	
			
	def Table1_move(self):
		print("Move Table 1")
		movebase_client(0)
			
		return
		
		#palette  = QPalette()
		#palette.setColor(QPalette.Highlight, Qt.yellow)     # default ==> Qt.darkBlue
		#palette.setColor(QPalette.HighlightedText, Qt.red)  # default ==> Qt.white
		#self.waypoint_tableWidget.setPalette(palette)
		
		
	
	def Table2_move(self):
		print("Move Table 2")
		movebase_client(1)
		row = 0		
		for col in range(self.waypoint_tableWidget.columnCount()):    
			self.waypoint_tableWidget.item(row, col).setBackground(QColor(Qt.gray))
		return
		
	def Table3_move(self):
		print("Move Table 3")
		movebase_client(2)
		return
	
	def Table4_move(self):
		print("Move Table 4")
		movebase_client(3)
		return

	def Table5_move(self):
		print("Move Table 5")		
		movebase_client(4)
		return

	def Table6_move(self):
		print("Move Table 6")		
		movebase_client(5)
		return

	def Table7_move(self):
		print("Move Table 7")		
		movebase_client(6)
		return

	def Table8_move(self):
		print("Move Table 8")
		movebase_client(7)
		return

	def Table9_move(self):
		print("Move Table 9")
		movebase_client(8)
		return

	def Table10_move(self):
		print("Move Table 10")
		movebase_client(9)
		return
		
	def Home_move(self):
		print("Move Home")
		return

	def Charge_move(self):
		print("Move Charge Station")
		return
		
if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()
