# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

import numpy as np
import math
import utm

X        = np.zeros(800)
Y        = np.zeros(800)
Yaw      = np.zeros(800)
no_line  = -1;

def read_gps_waypoints_data( ):
	fname = "/home/kyeongseo/one_fifth_catkin_ws/src/GPS_Package/GPS_GUI/scripts/GPS_Waypoint_Ui/track_1.txt"
	print(fname)
	f = open(fname, 'r')
	global X,Y,Yaw
	global no_line
	read_lines= f.readlines()
	no_line = len(read_lines)
	
	read_lines = list(map(lambda s: s.strip(), read_lines))
	
	#print(read_lines)

	for i in range(no_line):
		print(read_lines[i])		
		Data = read_lines[i].split()
		X[i]   = float(Data[0])
		Y[i]   = float(Data[1])
		Yaw[i] = float(Data[2])


def gps_data_plot( ):
	global X,Y
	global no_line
	X_data = np.zeros(no_line)
	Y_data = np.zeros(no_line)
	
	for i in range(no_line):
		
		X_data[i] = (float(X[i]) - float(X[0]) )  # 첫 번째 열을 x 좌표로 변환하여 추가
		Y_data[i] = (float(Y[i]) - float(Y[0]) )  # 두 번째 열을 y 좌표로 변환하여 추가
    
	for j in range(no_line):    
		print(X_data[j], Y_data[j])
	
	plt.plot(X_data, Y_data,marker='o',linestyle='-', color='b', label='data')
	plt.xlabel('X : UTM east')  # X 축 라벨 설정 (선택 사항)
	plt.ylabel('Y : UTM north')  # Y 축 라벨 설정 (선택 사항)
	plt.title('GPS Waypoints Graph')  # 그래프 제목 설정 (선택 사항)
	
	
	for j in range(no_line):    
		height = Y_data[j]
		plt.text(X_data[j], height + 0.25, '%d' %(j+1), ha='center', va='bottom', size = 8)
	plt.legend()	
	plt.show()

def main():
	print("test")	
	read_gps_waypoints_data()
	gps_data_plot( )
	
if __name__ == "__main__":
	
	main()		
