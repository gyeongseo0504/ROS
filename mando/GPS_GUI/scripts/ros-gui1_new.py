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

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import * 

waypoints_no = -1;
waypoint_start_id = -1
waypoint_finish_id = -1

def read_gps_waypoints_data(self):
	i =0;	
	f=open("//home//amap//amap//waypoints//waypoints_data.txt","r")
	lines = f.readlines()
	lines_num = len(lines)
	print(lines_num)
	gpswaypoints_text =""
	self.textEdit.setPlainText(gpswaypoints_text)		
	
	text_1 = 'No. of Waypoints : '
	text_1 += str(lines_num-1)
	
	self.label_waypoint_no.setText(text_1)
	if lines_num >2 :
		gps_waypoints_data = np.zeros((lines_num-1, 2))
	
	
	for line in lines:
		if i==0 :
			waypoints_no = int(line)
			print(waypoints_no)		
			
		else:
			lon, lat = line.split()
			gps_waypoints_data[i-1][0] = float(lon)
			gps_waypoints_data[i-1][1] = float(lat)
			gpswaypoints_text +="%8.5f    %8.5f\n" %(float(lon), float(lat))
		 
			#print(line, end='')	
		i += 1
	print(gps_waypoints_data)
	f.close()
	#self.listWidget_gps_waypoints.setPlainText(gpswaypoints_text)
	self.textEdit.setPlainText(gpswaypoints_text)
	
def pubish_waypoint_start_id(input_id):
	pub=rospy.Publisher('/start_waypoint_id_no', Int16, queue_size=1)	
	pub.publish(input_id)	
		
def pubish_waypoint_finish_id(input_id):
	pub=rospy.Publisher('/finish_waypoint_id_no', Int16, queue_size=1)		
	pub.publish(input_id)
	
def pubish_waypoint_start_topic():
	pub_run=rospy.Publisher('/waypoint_run_flag', Int8, queue_size=1)		
	pub_run.publish(1);
	
def pubish_waypoint_stop_topic():
	pub_run=rospy.Publisher('/waypoint_run_flag', Int8, queue_size=1)		
	pub_run.publish(0);
	
def pubish_path_reset_topic():
	pub_run=rospy.Publisher('/reset_path', Bool, queue_size=1)
	pub_run.publish(1);
	
	
	
	
#self.pushButton_Read_WPs.clicked.connect(self.button_read_gps_data_clicked)
#self.pushButton_Start_WP.clicked.connect(self.button_Start_WP_clicked)
#self.pushButton_Start_WP.clicked.connect(self.button_Start_WP_clicked)


class Ui_Dialog(object):
	
    def __init__(self):
	rospy.init_node('ROSgui', anonymous=True)
	
	# GPS Fix_status
	rospy.Subscriber('/gps/fix_status1', Bool, self.subGPS1FixStatusCallback)
	rospy.Subscriber('/gps/fix_status2', Bool, self.subGPS2FixStatusCallback)
	
	# GPS Fix_data
	rospy.Subscriber('/ublox_gps/fix',  NavSatFix, self.subGPS1FixDataCallback)
	rospy.Subscriber('/ublox_gps2/fix', NavSatFix, self.subGPS2FixDataCallback)
	
	# GPS Yaw Angle
	rospy.Subscriber('/gps_heading_angle', Float32, self.subGPSHeadingAngleCallback)
	
	print("Start Program")

    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(983, 570)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setGeometry(QtCore.QRect(610, 520, 341, 32))
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(30, 20, 281, 511))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton_Read_WPs = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_Read_WPs.setObjectName("pushButton_Read_WPs")
        self.verticalLayout.addWidget(self.pushButton_Read_WPs)
        self.label_waypoint_no = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_waypoint_no.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_waypoint_no.setObjectName("label_waypoint_no")
        self.verticalLayout.addWidget(self.label_waypoint_no)
        self.listWidget_gps_waypoints = QtWidgets.QListWidget(self.verticalLayoutWidget)
        self.listWidget_gps_waypoints.setObjectName("listWidget_gps_waypoints")
        self.verticalLayout.addWidget(self.listWidget_gps_waypoints)
        self.groupBox = QtWidgets.QGroupBox(Dialog)
        self.groupBox.setGeometry(QtCore.QRect(320, 20, 231, 311))
        self.groupBox.setObjectName("groupBox")
        self.label_gps1 = QtWidgets.QLabel(self.groupBox)
        self.label_gps1.setGeometry(QtCore.QRect(10, 30, 181, 17))
        self.label_gps1.setObjectName("label_gps1")
        self.label_gps2 = QtWidgets.QLabel(self.groupBox)
        self.label_gps2.setGeometry(QtCore.QRect(10, 80, 181, 17))
        self.label_gps2.setObjectName("label_gps2")
        self.label_gps_data1 = QtWidgets.QLabel(self.groupBox)
        self.label_gps_data1.setGeometry(QtCore.QRect(10, 50, 211, 17))
        self.label_gps_data1.setObjectName("label_gps_data1")
        self.label_gps_data2 = QtWidgets.QLabel(self.groupBox)
        self.label_gps_data2.setGeometry(QtCore.QRect(10, 100, 211, 17))
        self.label_gps_data2.setObjectName("label_gps_data2")
        self.label_yaw_angle = QtWidgets.QLabel(self.groupBox)
        self.label_yaw_angle.setGeometry(QtCore.QRect(10, 130, 171, 17))
        self.label_yaw_angle.setObjectName("label_yaw_angle")


        self.verticalLayoutWidget_2 = QtWidgets.QWidget(Dialog)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(565, 210, 141, 81))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.pushButton_Start_WP = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_Start_WP.setObjectName("pushButton_Start_WP")
        self.verticalLayout_2.addWidget(self.pushButton_Start_WP)
        self.pushButton_Finish_WP = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_Finish_WP.setObjectName("pushButton_Finish_WP")
        self.verticalLayout_2.addWidget(self.pushButton_Finish_WP)
        self.verticalLayoutWidget_3 = QtWidgets.QWidget(Dialog)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(710, 210, 57, 81))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.lineEdit_Start_WP_ID = QtWidgets.QLineEdit(self.verticalLayoutWidget_3)
        self.lineEdit_Start_WP_ID.setObjectName("lineEdit_Start_WP_ID")
        self.verticalLayout_3.addWidget(self.lineEdit_Start_WP_ID)
        self.lineEdit_Finish_WP_ID = QtWidgets.QLineEdit(self.verticalLayoutWidget_3)
        self.lineEdit_Finish_WP_ID.setObjectName("lineEdit_Finish_WP_ID")
        self.verticalLayout_3.addWidget(self.lineEdit_Finish_WP_ID)
        self.verticalLayoutWidget_4 = QtWidgets.QWidget(Dialog)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(560, 40, 211, 141))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.pushButton_Start = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.pushButton_Start.setObjectName("pushButton_Start")
        self.verticalLayout_4.addWidget(self.pushButton_Start)
        self.pushButton_Stop = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.pushButton_Stop.setObjectName("pushButton_Stop")
        self.verticalLayout_4.addWidget(self.pushButton_Stop)
        self.pushButton_RESET_PATH = QtWidgets.QPushButton(self.verticalLayoutWidget_4)
        self.pushButton_RESET_PATH.setObjectName("pushButton_RESET_PATH")
        self.verticalLayout_4.addWidget(self.pushButton_RESET_PATH)
        self.textEdit = QtWidgets.QTextEdit(Dialog)
        self.textEdit.setGeometry(QtCore.QRect(320, 350, 279, 171))
        self.textEdit.setObjectName("textEdit")
                
        self.pushButton_Read_WPs.clicked.connect(self.button_read_gps_data_clicked)        
        self.pushButton_Start_WP.clicked.connect(self.button_Start_WP_clicked)    
        self.pushButton_Finish_WP.clicked.connect(self.button_Finish_WP_clicked)
        self.pushButton_RESET_PATH.clicked.connect(self.button_Reset_Path_clicked)
        self.pushButton_Start.clicked.connect(self.button_Start_clicked)
        self.pushButton_Stop.clicked.connect(self.button_Stop_clicked)
        
        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
        
    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Test"))
        self.pushButton_Read_WPs.setText(_translate("Dialog", "Read Waypoints"))
        self.label_waypoint_no.setText(_translate("Dialog", "No. of Waypoints :"))
        self.groupBox.setTitle(_translate("Dialog", "GPS Data  "))
        self.label_gps1.setText(_translate("Dialog", "GPS 1 :  Float"))
        self.label_gps2.setText(_translate("Dialog", "GPS 2 :  Float"))
        self.label_gps_data1.setText(_translate("Dialog", "Lon :                     Lati :"))
        self.label_gps_data2.setText(_translate("Dialog", "Lon :                     Lati :"))
        self.label_yaw_angle.setText(_translate("Dialog", "Yaw Angle :"))
        self.pushButton_Start_WP.setText(_translate("Dialog", "Start Waypoint ID"))
        self.pushButton_Finish_WP.setText(_translate("Dialog", "Finish Waypoint ID"))
        self.lineEdit_Start_WP_ID.setText(_translate("Dialog", "0"))
        self.lineEdit_Finish_WP_ID.setText(_translate("Dialog", "10"))
        self.pushButton_Start.setText(_translate("Dialog", "Start Run"))
        self.pushButton_Stop.setText(_translate("Dialog", "Stop"))
        self.pushButton_RESET_PATH.setText(_translate("Dialog", "Reset Path"))
		
    			
    def button_read_gps_data_clicked(self):
		read_gps_waypoints_data(self)
		
    def button_Start_clicked(self):
		self.pushButton_Start.setDisabled(True)
		self.pushButton_Stop.setEnabled(True)
		pubish_waypoint_start_topic()
		
    def button_Stop_clicked(self):
		self.pushButton_Start.setEnabled(True)	
		self.pushButton_Stop.setDisabled(True)
		pubish_waypoint_stop_topic()	
		
		
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
		
    def button_Reset_Path_clicked(self):	
		print("Reset Path ")
		pubish_path_reset_topic()
		
    def subGPS1FixStatusCallback(self,fix_data):
		if fix_data.data == 1 :
			self.label_gps1.setText("GPS 1 :  Fix  ")
			#print("GSP1 Fix")
		else:
			self.label_gps1.setText("GPS 1 :  Float") 
			#print("GSP1 Float")	

    def subGPS2FixStatusCallback(self,fix_data):
		if fix_data.data == 1 :
			self.label_gps2.setText("GPS 2 :  Fix  ")
			#print("GSP1 Fix")
		else:
			self.label_gps2.setText("GPS 2 :  Float") 
			#print("GSP1 Float")	
    
    def subGPSHeadingAngleCallback(self,y_angle):
		self.label_yaw_angle.setText("Yaw Angle : {:5.2f} ".format(y_angle.data*180.0/math.pi))
		print(y_angle.data)
    
    def subGPS1FixDataCallback(self, gps_fix):
		self.label_gps_data1.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude,gps_fix.longitude))
		
    def subGPS2FixDataCallback(self, gps_fix):
		self.label_gps_data2.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude,gps_fix.longitude))	
		      

   
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_()) 

    

	
    
 
