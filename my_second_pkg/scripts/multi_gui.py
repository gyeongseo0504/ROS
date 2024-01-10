
from __future__ import print_function


import math
import os,sys
import rospy
import time

print(sys.version)
import warnings
warnings.filterwarnings('ignore')


from std_msgs.msg      import String
from std_msgs.msg      import Int16
from std_msgs.msg      import Int8
from std_msgs.msg      import Bool
from std_msgs.msg      import Float32

from PyQt5.QtCore      import *
from PyQt5.QtCore      import Qt
from PyQt5.QtWidgets   import *
from PyQt5             import uic
from pyqtgraph         import PlotWidget
from PyQt5.QtWidgets   import QWidget, QTabWidget, QAction
from PyQt5.QtGui       import *
from PyQt5.QtCore 	   import QEvent


import pyqtgraph       as pg

import matplotlib.pyplot as plt

import numpy as np
import math


form_class = uic.loadUiType('multiplication_table.ui')[0]

def publish_topic_x(x):
	pub=rospy.Publisher('/float32/x', Float32, queue_size=1)	
	print("publish_parameter x: ",x)
	pub.publish(x)
	return	
	
def publish_topic_y(y):
	pub=rospy.Publisher('/float32/y', Float32, queue_size=1)	
	print("publish_parameter y: ",y)
	pub.publish(y)
	return		


def publish_operator(x):
	pub=rospy.Publisher('/string/operator', String, queue_size=1)	
	print("publish_operator : ",x)
	pub.publish(x)
	return	
		
	
	

class WindowClass(QDialog, form_class):
	
	
	def __init__(self):
		
		super(QDialog,self).__init__()
		
		rospy.init_node('ROS_train_gui', anonymous=True)
		
		self.setupUi(self)
		self.setWindowTitle("ROS train GUI")
		
		self.pushButton_publish.clicked.connect(self.publish_topic)
		
	def publish_topic(self):
		print("Published")
		
		x = float(self.lineEdit_pose_x.text())
		publish_topic_x(x)
		
		list_text = self.comboBox_operator.currentText()
		publish_operator(list_text)
		
		y = float(self.lineEdit_pose_y.text())
		publish_topic_y(y)
		
		return	
		
if __name__ == "__main__":
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()		
		
