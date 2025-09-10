# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!


from __future__ import print_function


import math
import os,sys
import rospy

print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from pyqtgraph import PlotWidget
import pyqtgraph as pg

import matplotlib.pyplot as plt

import numpy as np
import math
import utm


form_class = uic.loadUiType('gps_test.ui')[0]

print(form_class) 

GPS_Fix  = np.zeros(2)
X        = np.zeros(200)
Y        = np.zeros(200)
Yaw      = np.zeros(200)
waypoint_start_id   = 0
waypoint_finish_id  = 10
no_line = -1
heading_angle_gps = 0.0
heading_angle_topic = 0.0

gps1_utm = np.zeros(2)
gps2_utm = np.zeros(2)

def pubish_reset_path():
	pub=rospy.Publisher('/reset_path', Bool, queue_size=1)	
	pub.publish(1)	
	
def pubish_waypoint_start_id(input_id):
	pub=rospy.Publisher('/start_waypoint_id_no', Int16, queue_size=1)	
	pub.publish(input_id)	
		
def pubish_waypoint_finish_id(input_id):
	pub=rospy.Publisher('/finish_waypoint_id_no', Int16, queue_size=1)		
	pub.publish(input_id)
	
def pubish_waypoint_start_topic():
	pub_run=rospy.Publisher('/wp/waypoint_run_flag', Int8, queue_size=1)		
	pub_run.publish(1);
	
def pubish_waypoint_stop_topic():
	pub_run=rospy.Publisher('/wp/waypoint_run_flag', Int8, queue_size=1)		
	pub_run.publish(0);
	
class WindowClass(QDialog, form_class):
	def __init__(self):
		
		super(QDialog,self).__init__()
		
		rospy.init_node('ROS_gps_gui', anonymous=True)

		 #GPS Fix_status
		rospy.Subscriber('/gps/fix_status1', Bool, self.subGPS1FixStatusCallback)
		rospy.Subscriber('/gps/fix_status2', Bool, self.subGPS2FixStatusCallback)
		 
		 # GPS Fix_data	
		#rospy.Subscriber('/fix',  NavSatFix, self.subGPS1FixDataCallback)
		rospy.Subscriber('/gps1/fix',  NavSatFix, self.subGPS1FixDataCallback)
		rospy.Subscriber('/gps2/fix', NavSatFix, self.subGPS2FixDataCallback)
		 
		 # GPS Yaw Angle
		rospy.Subscriber('/gps_heading_angle', Float32, self.subGPSHeadingAngleCallback)		
				
		self.setupUi(self)
		
		self.setWindowTitle("AMR GUI")
		
		self.waypoint_tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # row 단위로 선택 가능
		self.pushButton_Read_WPs.clicked.connect(self.read_gps_waypoints_data)
		self.waypoint_tableWidget.cellClicked.connect(self.select_tableWidget_event)
		self.pushButton_remove_waypoint.clicked.connect(self.remove_waypoint)
		self.pushButton_insert_waypoint.clicked.connect(self.insert_waypoint)
		self.pushButton_save_waypoints.clicked.connect(self.save_waypoints)
		self.pushButton_Plot_WPs.clicked.connect(self.Plot_GPS_Waypoints)
		self.pushButton_Start_WP.clicked.connect(self.button_Start_WP_clicked)
		self.pushButton_Finish_WP.clicked.connect(self.button_Finish_WP_clicked)
		
		self.pushButton_Start.clicked.connect(self.button_Start_clicked)
		self.pushButton_Stop.clicked.connect(self.button_Stop_clicked)
		self.pushButton_RESET_PATH.clicked.connect(self.button_reset_clicked)
		self.pushButton_clear_waypoints.clicked.connect(self.remove_all_waypoints)
				
		self.lineEdit_Start_WP_ID.textChanged.connect(self.Start_WP_changed)
		self.lineEdit_Finish_WP_ID.textChanged.connect(self.Finish_WP_changed)
		
		
		
		self.pushButton_Start.setEnabled(True)
		self.pushButton_Stop.setDisabled(True)
		self.pushButton_Plot_WPs.setDisabled(True)
		
		
	def plot_gps_waypoints_graphWidget(self):
		
		global X,Y
		no_data = self.waypoint_tableWidget.rowCount();
		print("No of WayPoints Data ", no_data);
		if no_data >= 1 :
				X_data = np.zeros(no_data)
				Y_data = np.zeros(no_data)
				for i in range(no_data):
					X_data[i] = X[i]-X[0]
					Y_data[i] = Y[i]-Y[0]
					
				self.graphWidget.setTitle("GPS WayPoints")	
				pen = pg.mkPen(color=(255, 0, 0))	
				self.graphWidget.clear()  
				self.graphWidget.setBackground('w')	
				self.graphWidget.plot(X_data,Y_data,pen=pen)
		return
					
		
	def read_gps_waypoints_data(self):
		print("read GPS WayPoints")
		global no_line
		fname = QFileDialog.getOpenFileName(self, 'Open file', './')
		
		if fname[0]:
			print(fname[0])
			f = open(fname[0], 'r')
			self.label_file_name.setText(fname[0])
			read_lines= f.readlines()
			no_line = len(read_lines)
			read_lines = list(map(lambda s: s.strip(), read_lines))
			self.waypoint_tableWidget.setRowCount(no_line)
			print(read_lines)
			
			for i in range(no_line):
				print(read_lines[i])
				
				Data = read_lines[i].split()
				X[i]   = float(Data[0])
				Y[i]   = float(Data[1])
				
				#coordinate = utm.from_latlon(X[i],Y[i])
				
				self.waypoint_tableWidget.setItem(i,0,QTableWidgetItem(Data[0]))
				self.waypoint_tableWidget.item(i, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
				self.waypoint_tableWidget.setItem(i,1,QTableWidgetItem(Data[1]))
				self.waypoint_tableWidget.item(i, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
						
			
			self.waypoint_tableWidget.resizeColumnToContents(0)
			self.waypoint_tableWidget.resizeColumnToContents(1)
			self.plot_gps_waypoints_graphWidget()
							
			self.lineEdit_Finish_WP_ID.setText(str(no_line))
			f.close()
			
			self.pushButton_Start.setEnabled(True)			
			self.pushButton_Stop.setDisabled(True)
			self.pushButton_Plot_WPs.setEnabled(True)
			return
	
	def select_tableWidget_event(self):
		print("selected")
		global no_line
		self.waypoint_tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # row 단위로 선택 가능
		row = self.waypoint_tableWidget.currentRow()
		no_line = row
		print(row)
		return
	
	def remove_waypoint(self):
		print("remvove waypoint")
		row = self.waypoint_tableWidget.currentRow()
		self.waypoint_tableWidget.removeRow(row)  # column 삭제
		return
	
	def remove_all_waypoints(self):
		print("remvove all waypoints")		
		global no_line
		self.waypoint_tableWidget.clear()  # column 삭제		
		no_line = 0
		return	
		
	def update_gps_waypoints(self):
		print("update gps waypoints")
		
		data_no= self.waypoint_tableWidget.rowCount();
		print("data no ", data_no)
		
		for i in range(data_no):
			data1 = self.waypoint_tableWidget.item(i,0)
			
			if data1 is not None :
				X[i] = float( data1.text() ) 
				
			data2 = self.waypoint_tableWidget.item(i,1)
			if data2 is not None :
				Y[i] = float( data2.text() ) 
				
			print(i, X[i], Y[i])
		
		self.plot_gps_waypoints_graphWidget()		
					
		return
	   	
		
	def Plot_GPS_Waypoints(self):
		print("plot waypoints")
		data_no= self.waypoint_tableWidget.rowCount();
		print("data no : ", data_no)
		global X,Y
	
		for i in range(data_no):
			print(i, "  X : Y",X[i],Y[i])
		
		
		if data_no >= 1 :
			X_data = np.zeros(data_no)
			Y_data = np.zeros(data_no)
			for i in range(data_no):
				X_data[i] = X[i]-X[0]
				Y_data[i] = Y[i]-Y[0]
				
			plt.scatter(X_data,Y_data)
			plt.title("GPS Waypoints")
			plt.show()
			
		return	
		
	def insert_waypoint(self):
		print("insert waypoint")
		global no_line
		global X,Y
		#print(gps1_utm[0], gps1_utm[1])
		row = self.waypoint_tableWidget.currentRow()
		
		self.waypoint_tableWidget.resizeColumnToContents(0)
		self.waypoint_tableWidget.resizeColumnToContents(1) 
		
		no_line = self.waypoint_tableWidget.rowCount();
		row  = row + 1
		print("row : " ,row ,"no_line", no_line)
		self.waypoint_tableWidget.insertRow(row)  # row 추가			
		self.waypoint_tableWidget.setItem(row,0,QTableWidgetItem(str(gps1_utm[0])))
		self.waypoint_tableWidget.item(row, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
		self.waypoint_tableWidget.setItem(row,1,QTableWidgetItem(str(gps1_utm[1])))				
		self.waypoint_tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
		X[row] = gps1_utm[0]
		Y[row] = gps1_utm[1]
				
		self.update_gps_waypoints()
		self.pushButton_Plot_WPs.setEnabled(True)
		return
		
	
	
	def button_reset_clicked(self):
		print("Reset Path")
		pubish_reset_path()
		return
	
		
			
	def save_waypoints(self):  # save waypoints 
		print("save waypoint")
		
		data_no= self.waypoint_tableWidget.rowCount();
		print("row count :",data_no)
		fname = QFileDialog.getSaveFileName(self, 'Save file', './')
		if fname[0]:
			print(fname[0])
			f = open(fname[0], 'w')
			for i in range(data_no):
				data1 = self.waypoint_tableWidget.item(i,0)
				if data1 is not None :
					f.write( data1.text() )
										
				f.write("  ")
				
				data2 = self.waypoint_tableWidget.item(i,1)
				if data2 is not None :					
					f.write( data2.text() )
				f.write("\n")				
				print(data1.text(),"  ", data2.text())				
							
			f.close()
		return
	
	def Start_WP_changed(self):	
		input_id = self.lineEdit_Start_WP_ID.text()				
		waypoint_start_id = int(input_id)
		print("wayoint start id ",input_id)	
		return
		
	def Finish_WP_changed(self):	
		input_id = self.lineEdit_Finish_WP_ID.text()			
		waypoint_finish_id = int(input_id)
		print("wayoint finish id ",waypoint_finish_id)	
		return
		
	def button_Start_WP_clicked(self):
		input_id = self.lineEdit_Start_WP_ID.text()				
		waypoint_start_id = int(input_id)
		print("wayoint start id ",input_id)	
		pubish_waypoint_start_id(waypoint_start_id)
		
	def button_Finish_WP_clicked(self):
		input_id = self.lineEdit_Finish_WP_ID.text()			
		waypoint_finish_id = int(input_id)
		print("wayoint finish id ",waypoint_finish_id)		
		pubish_waypoint_finish_id(waypoint_finish_id)   
	
	def button_Start_clicked(self):
		self.pushButton_Start.setDisabled(True)
		self.pushButton_Stop.setEnabled(True)
		pubish_waypoint_start_topic()
		
	def button_Stop_clicked(self):
		self.pushButton_Start.setEnabled(True)	
		self.pushButton_Stop.setDisabled(True)
		pubish_waypoint_stop_topic()
			
	def subGPS1FixStatusCallback(self,fix_data):
		if fix_data.data == 1 :
			self.label_gps1.setText("GPS 1 :  Fix  ")
			GPS_Fix[0] = 1;
			print("GSP1 Fix")
		else:
			self.label_gps1.setText("GPS 1 :  Float") 
			GPS_Fix[0] = 0;
			print("GSP1 Float")
		return	
	
	def subGPS2FixStatusCallback(self,fix_data):
		if fix_data.data == 1 :
			self.label_gps2.setText("GPS 2 :  Fix  ")
			GPS_Fix[1] = 1;
			print("GSP2 Fix")
		else:
			self.label_gps2.setText("GPS 2 :  Float") 
			GPS_Fix[1] = 0;
			print("GSP2 Float")
				
	def subGPSHeadingAngleCallback(self,y_angle):
		
		heading_angle_topic =  y_angle.data
		self.label_yaw_angle_1.setText("Yaw Angle : {:5.2f} ".format(heading_angle_topic*180.0/math.pi) ) 
	    
		print(y_angle.data)
		return
	
	def subGPS1FixDataCallback(self, gps_fix):
		self.label_gps_data1.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude,gps_fix.longitude))		
		coordinate = utm.from_latlon(gps_fix.latitude, gps_fix.longitude)
		gps1_utm[0] = coordinate[0]
		gps1_utm[1] = coordinate[1]
		self.calculate_yaw_angle()
		#print(gps1_utm[0], gps1_utm[1])
		return
	
	#이 부분은 검정 해야 함
	def calculate_yaw_angle(self):
		global heading_angle_gps
		global gps1_utm ,gps2_utm 
		print("Calculate GPS Angle")
		if (GPS_Fix[0] == 1) & (GPS_Fix[1] == 1) :
			dis_x = gps1_utm[0] - gps2_utm[0]
			dis_y = gps1_utm[1] - gps2_utm[1]
			
			angle = -math.atan2(dis_x, dis_y)*180.0/math.pi
			#self.label_yaw_angle.setText("Yaw Angle : {:5.2f} ".format(angle))
			self.label_yaw_angle_2.setText("Yaw Angle(sim) : {:5.2f} ".format(angle))
			print("GPS Heading Angle",angle)			
		
		return
		
	
	def subGPS2FixDataCallback(self, gps_fix):
		self.label_gps_data2.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude,gps_fix.longitude))			
		coordinate = utm.from_latlon(gps_fix.latitude, gps_fix.longitude)
		gps2_utm[0] = coordinate[0]
		gps2_utm[1] = coordinate[1]
		return

if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()
